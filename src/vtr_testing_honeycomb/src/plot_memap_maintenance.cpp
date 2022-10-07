#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/data_types/multi_exp_pointmap.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"
#include "vtr_lidar/pipeline.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/path/pose_cache.hpp"
#include "vtr_tactic/modules/factory.hpp"

#include "vtr_testing_honeycomb/utils.hpp"

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::lidar;
using namespace vtr::testing;

// clang-format off

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  rclcpp::init(argc, argv);
  const std::string node_name = "plot_map_maintenance_" + random_string(5);
  auto node = rclcpp::Node::make_shared(node_name);

  // Output directory
  const auto data_dir_str = node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Configure logging
  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled = node->declare_parameter<std::vector<std::string>>("log_enabled", std::vector<std::string>{});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  //
  auto global_path_pub = node->create_publisher<nav_msgs::msg::Path>("global_path", 5);
  auto global_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 5);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);
  tactic::VertexId priv_vid(0, 0);

  /// Create a temporal evaluator
  auto evaluator = std::make_shared<tactic::TemporalEvaluator<tactic::GraphBase>>(*graph);
  auto subgraph = graph->getSubgraph(priv_vid, evaluator);

  /// global map for visualization
  std::vector<Eigen::Matrix4d> T_priv_currs;
  PointMap<PointWithInfo> global_map(/* voxel size */ 0.2);
  pose_graph::PoseCache<GraphBase> pose_cache(subgraph, priv_vid);

  for (auto it = subgraph->begin(priv_vid); it != subgraph->end(); ++it) {
    const auto curr_vertex = it->v();
    const auto curr_vid = it->v()->id();


    // store the privileged path in global frame
    {
      T_priv_currs.emplace_back(pose_cache.T_root_query(curr_vid).matrix());
    }

    // check if we have a map for this vertex
    {
      const auto msg = curr_vertex->retrieve<PointMapPointer>("pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
      const auto pointmap_ptr = msg->sharedLocked().get().getData();
      if (pointmap_ptr.map_vid != curr_vid)
        continue;
    }

    {
      // load map
      const auto map_msg = curr_vertex->retrieve<MultiExpPointMap<PointWithInfo>>("mepointmap", "vtr_lidar_msgs/msg/MultiExpPointMap");
      auto curr_pointmap = map_msg->sharedLocked().get().getData();
      auto curr_pointcloud = curr_pointmap.point_cloud();

      {
        // transform to its vertex to remove points
        auto points_mat = curr_pointcloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
        auto T_curr_map = curr_pointmap.T_vertex_this().matrix().cast<float>();
        points_mat = T_curr_map * points_mat;
#if false
        // remove points on the ground
        auto filter_cb = [](PointWithInfo &query_pt) {
          return bool(query_pt.z > 0.6);
        };
        std::vector<int> indices;
        indices.reserve(curr_pointcloud.size());
        for (size_t i = 0; i < curr_pointcloud.size(); ++i) {
          if (filter_cb(curr_pointcloud[i])) indices.emplace_back(i);
        }
        pcl::copyPointCloud(curr_pointcloud, indices, curr_pointcloud);
#endif
#if true
        // store z value
        for (auto &pt : curr_pointcloud) pt.flex22 = pt.z;
#endif
      }
      {
        // transform to the local frame of this vertex
        auto points_mat = curr_pointcloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
        auto T_priv_curr = pose_cache.T_root_query(curr_vid).matrix().cast<float>();
        points_mat = T_priv_curr * points_mat;
      }

      // update the map
      auto update_cb = [](bool, PointWithInfo &curr_pt,
                          const PointWithInfo &new_pt) {
        if (new_pt.multi_exp_obs > curr_pt.multi_exp_obs) curr_pt = new_pt;
      };
      global_map.update(curr_pointcloud, update_cb);
    }

    // memory management
    graph->at(curr_vid)->unload();
  }

#if false
  /// \note uncomment this to remove short-term obstacles (hardcoded for parking lot only)
  auto filter_cb = [](PointWithInfo &query_pt) {
    return bool(query_pt.multi_exp_obs > 18);
  };
  global_map.filter(filter_cb);
#endif

  /// publish the global map
  while (true) {
    if (!rclcpp::ok()) break;
    CLOG(WARNING, "test") << "Publishing global path and map.";

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "world";
    auto& poses = path_msg.poses;
    for (auto &T_priv_curr: T_priv_currs) {
      auto& pose = poses.emplace_back();
      pose.pose = tf2::toMsg(Eigen::Affine3d(T_priv_curr));
    }
    global_path_pub->publish(path_msg);

    sensor_msgs::msg::PointCloud2 pc2_msg;
    pcl::toROSMsg(global_map.point_cloud(), pc2_msg);
    pc2_msg.header.frame_id = "world";
    // pc2_msg.header.stamp = 0;
    global_map_pub->publish(pc2_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  rclcpp::shutdown();

  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";
}