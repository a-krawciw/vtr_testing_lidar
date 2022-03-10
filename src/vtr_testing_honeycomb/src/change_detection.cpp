#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "std_msgs/msg/bool.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"
#include "vtr_lidar/pipeline.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/modules/factory.hpp"

#include "vtr_testing_honeycomb/utils.hpp"

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::lidar;
using namespace vtr::testing;

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  rclcpp::init(argc, argv);
  const std::string node_name = "change_detection_" + random_string(5);
  auto node = rclcpp::Node::make_shared(node_name);

  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Configure logging
  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled = node->declare_parameter<std::vector<std::string>>(
      "log_enabled", std::vector<std::string>{});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  // world offset for localization path visualization
  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  Eigen::Vector3d vis_loc_path_offset;
  vis_loc_path_offset << 0.0, 0.0, 0.0;
  Eigen::Affine3d T(Eigen::Translation3d{vis_loc_path_offset});
  auto msg = tf2::eigenToTransform(T);
  msg.header.frame_id = "world";
  msg.child_frame_id = "world (offset)";
  tf_sbc->sendTransform(msg);

  // Parameters
  const unsigned run_id = node->declare_parameter<int>("run_id", 1);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

  // module
  auto module_factory = std::make_shared<ROSModuleFactory>(node);
  auto module = module_factory->get("localization.change_detection");

  /// robot lidar transformation is hard-coded - check measurements.
  Eigen::Matrix4d T_lidar_robot_mat;
  T_lidar_robot_mat << 1, 0, 0, -0.06, 0, 1, 0, 0, 0, 0, 1, -1.45, 0, 0, 0, 1;
  EdgeTransform T_lidar_robot(T_lidar_robot_mat);
  T_lidar_robot.setZeroCovariance();

  // thread handling variables
  TestControl test_control(node);

  size_t depth = 10;
  std::queue<tactic::VertexId> ids;

  /// Create a temporal evaluator
  auto evaluator =
      std::make_shared<tactic::TemporalEvaluator<tactic::GraphBase>>(*graph);

  auto subgraph = graph->getSubgraph(tactic::VertexId(run_id, 0), evaluator);
  for (auto it = subgraph->begin(tactic::VertexId(run_id, 0));
       it != subgraph->end();) {
    /// test control
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(test_control.delay()));

    /// caches
    lidar::LidarQueryCache qdata;
    lidar::LidarOutputCache output;

    qdata.node = node;

    const auto vertex_odo = it->v();

    const auto scan_msg = vertex_odo->retrieve<PointScan<PointWithInfo>>(
        "filtered_point_cloud", "vtr_lidar_msgs/msg/PointScan");
    auto locked_scan_msg_ref = scan_msg->sharedLocked();  // lock the msg
    auto &locked_scan_msg = locked_scan_msg_ref.get();

    // get scan timestamp
    const auto stamp = locked_scan_msg.getTimestamp();
    qdata.stamp.emplace(stamp);

    // get T_s_r
    qdata.T_s_r.emplace(T_lidar_robot);

    // get undistorted lidar scan
    const auto &point_scan = locked_scan_msg.getData();
    const auto &point_cloud = point_scan.point_cloud();
    qdata.undistorted_point_cloud.emplace(point_cloud);

    // find the privileged vertex it has been localized against
    const auto neighbors = graph->neighbors(vertex_odo->id());
    VertexId vid_loc = VertexId::Invalid();
    for (auto neighbor : neighbors) {
      if (graph->at(EdgeId(vertex_odo->id(), neighbor))->isSpatial()) {
        vid_loc = neighbor;
        break;
      }
    }
    if (!vid_loc.isValid()) continue;

    // load the map pointer
    const auto pointmap_ptr = [&] {
      const auto vertex_loc = graph->at(vid_loc);
      const auto msg = vertex_loc->retrieve<PointMapPointer>(
          "pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
      auto locked_msg = msg->sharedLocked();
      return locked_msg.get().getData();
    }();

    auto vertex_loc_map = graph->at(pointmap_ptr.map_vid);
    const auto map_msg = vertex_loc_map->retrieve<PointMap<PointWithInfo>>(
        "pointmap", "vtr_lidar_msgs/msg/PointMap");
    if (map_msg == nullptr) {
      CLOG(ERROR, "test") << "Could not find map at vertex " << vid_loc;
      throw std::runtime_error("Could not find map at vertex " +
                               std::to_string(vid_loc));
    }
    auto locked_map_msg = map_msg->sharedLocked();
    qdata.submap_loc = std::make_shared<PointMap<PointWithInfo>>(
        locked_map_msg.get().getData());
    //
    qdata.T_v_m_loc.emplace(pointmap_ptr.T_v_this_map *
                            qdata.submap_loc->T_vertex_this());

    const auto &T_ov_s = point_scan.T_vertex_this();
    const auto &T_lv_ov = graph->at(EdgeId(vid_loc, vertex_odo->id()))->T();
    const auto &T_s_r = T_lidar_robot;
    const auto &T_r_lv = (T_lv_ov * T_ov_s * T_s_r).inverse();
    qdata.vid_loc.emplace(vid_loc);
    qdata.sid_loc.emplace(0);  /// \note: random sid since it is not used
    qdata.T_r_v_loc.emplace(T_r_lv);

    CLOG(WARNING, "test") << "Change detection for scan at stamp: " << stamp;
    CLOG(WARNING, "test") << "Loaded pointmap pointer with this_vid "
                          << pointmap_ptr.this_vid << " and map_vid "
                          << pointmap_ptr.map_vid;
    module->runAsync(qdata, output, graph, nullptr, {}, {});

    // memory management
    ids.push(it->v()->id());
    if (ids.size() > depth) {
      graph->at(ids.front())->unload();
      ids.pop();
    }

    // increment
    ++it;
  }

  rclcpp::shutdown();

  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
}