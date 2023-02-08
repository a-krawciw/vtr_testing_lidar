#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/int32.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/data_types/point.hpp"
#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/types.hpp"

#include "vtr_testing_honeycomb/utils.hpp"

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::lidar;
using namespace vtr::logging;
using namespace vtr::testing;

class GraphAnnotator {
 public:
  using Command = std_msgs::msg::Int32;
  using PointCloud = sensor_msgs::msg::PointCloud2;

  GraphAnnotator(const rclcpp::Node::SharedPtr &node, tactic::GraphBase &graph,
                 const tactic::VertexId &root)
      : graph_(graph), iter_(graph.begin(root)) {
    // create subscriptions
    // clang-format off
    pc_sub_ = node->create_subscription<PointCloud>("/selected_points", 10, std::bind(&GraphAnnotator::annotationCb, this, std::placeholders::_1));
    cmd_sub_ = node->create_subscription<Command>("/selection_command", 10, std::bind(&GraphAnnotator::commandCb, this, std::placeholders::_1));
    curr_map_pub_ = node->create_publisher<PointCloud>("/annotation_map", 5);
    // clang-format on

    // publish the initial map
    getNextMap();
    publishCurrMap();
  }

 private:
  void annotationCb(const PointCloud::SharedPtr point_cloud) {
    //
    CLOG(WARNING, "annotation") << "Annotating selected points";
    annotateMap(point_cloud);
    publishCurrMap();
    CLOG(WARNING, "annotation") << "Annotating selected points - DONE";
  }

  void commandCb(const Command::SharedPtr command) {
    auto cmd = command->data;
    //
    CLOG(WARNING, "annotation") << "";  // empty line
    std::stringstream ss;
    ss << "Received command: ";
    //
    switch (cmd) {
      case -2:
        // cancel the selection
        curr_type_ = 0;
        annotateMap();
        publishCurrMap();
        ss << "cancel selection";
        break;
      case -1:
        // proceed to the next point cloud
        saveCurrMap();
        getNextMap();
        publishCurrMap();
        ss << "proceed to next point cloud";
        break;
      case 0:
        curr_type_ = 0;
        ss << "type=0";
        break;
      case 1:
        curr_type_ = 1;
        ss << "type=1";
        break;
      case 2:
        curr_type_ = 2;
        ss << "type=2";
        break;
      default:
        break;
    }
    CLOG(WARNING, "annotation") << ss.str();
  }

 private:
  void saveCurrMap() {
    if (!curr_map_) return;
    //
    const auto vid = curr_map_->vertex_id();
    auto vertex = graph_.at(vid);
    //
    using PointMapLM = storage::LockableMessage<PointMap<PointWithInfo>>;
    auto msg = std::make_shared<PointMapLM>(curr_map_, vertex->vertexTime());
    vertex->insert<PointMap<PointWithInfo>>("pointmap_annotated",
                                            "vtr_lidar_msgs/msg/PointMap", msg);
    CLOG(WARNING, "annotation")
        << "Save annotated map with vertex id: " << curr_map_->vertex_id();
    curr_map_ = nullptr;
    curr_adapter_ = nullptr;
    curr_kdtree_ = nullptr;
  }

  void getNextMap() {
    for (; iter_ != graph_.end(); ++iter_) {
      //
      const auto vertex = iter_->v();

      // check if this vertex has a map
      {
        const auto msg = vertex->retrieve<PointMapPointer>(
            "pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
        auto locked_msg = msg->sharedLocked();
        const auto &pointmap_ptr = locked_msg.get().getData();
        if (pointmap_ptr.map_vid != vertex->id()) continue;
      }

      // retrieve point map v0 (initial map) from this vertex
      // clang-format off
      const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>("pointmap", "vtr_lidar_msgs/msg/PointMap");
      curr_map_ = std::make_shared<PointMap<PointWithInfo>>(map_msg->sharedLocked().get().getData());
      curr_adapter_ = std::make_unique<NanoFLANNAdapter<PointWithInfo>>(curr_map_->point_cloud());
      curr_kdtree_ = std::make_unique<KDTree<PointWithInfo>>(3, *curr_adapter_, KDTreeParams(10));
      curr_kdtree_->buildIndex();
      // clang-format on

      ++iter_;

      CLOG(WARNING, "annotation")
          << "Switched to map with vertex id: " << curr_map_->vertex_id();
      return;
    }
    //
    curr_map_ = nullptr;
    curr_adapter_ = nullptr;
    curr_kdtree_ = nullptr;
    CLOG(WARNING, "annotation") << "All point maps have been annotated.";
  }

  void publishCurrMap() {
    if (curr_map_ == nullptr) return;
    //
    CLOG(WARNING, "annotation")
        << "Publishing map with vertex id: " << curr_map_->vertex_id();
    PointCloud pc_msg;
    pcl::toROSMsg(curr_map_->point_cloud(), pc_msg);
    pc_msg.header.frame_id = "unknown";
    // pc_msg.header.stamp = 0;
    curr_map_pub_->publish(pc_msg);
  }

  void annotateMap(const PointCloud::SharedPtr pc_msg = nullptr) {
    if (curr_map_ == nullptr) return;
    //
    if (pc_msg == nullptr) {
      CLOG(WARNING, "annotation")
          << "Annotating the entire map to type: " << curr_type_;
      auto &point_cloud = curr_map_->point_cloud();
      for (auto &point : point_cloud) point.flex21 = (float)curr_type_;
    } else {
      CLOG(WARNING, "annotation")
          << "Annotating the selected points to type: " << curr_type_;
      //
      using IterType = sensor_msgs::PointCloud2ConstIterator<float>;
      IterType iter_x(*pc_msg, "x"), iter_y(*pc_msg, "y"), iter_z(*pc_msg, "z");
      //
      std::vector<size_t> inds;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        KDTreeResultSet nn(1);
        size_t ind = -1;
        float dist = 0;
        nn.init(&ind, &dist);
        float query[] = {*iter_x, *iter_y, *iter_z};
        curr_kdtree_->findNeighbors(nn, query, KDTreeSearchParams());
        if (dist > 0.1) {
          CLOG(ERROR, "annotation") << "Point is too far away: " << dist;
          throw std::runtime_error("distance too large");
        } else {
          inds.emplace_back(ind);
        }
      }
      //
      for (const auto &ind : inds)
        curr_map_->point_cloud().at(ind).flex21 = (float)curr_type_;
    }
  }

 private:
  rclcpp::Subscription<PointCloud>::SharedPtr pc_sub_;
  rclcpp::Subscription<Command>::SharedPtr cmd_sub_;
  rclcpp::Publisher<PointCloud>::SharedPtr curr_map_pub_;

  tactic::GraphBase &graph_;
  tactic::GraphBase::OrderedIter iter_;

  int curr_type_ = 0;

  std::shared_ptr<PointMap<PointWithInfo>> curr_map_;
  std::unique_ptr<NanoFLANNAdapter<PointWithInfo>> curr_adapter_;
  std::unique_ptr<KDTree<PointWithInfo>> curr_kdtree_;
};

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  rclcpp::init(argc, argv);
  const std::string node_name = "point_annotation_" + random_string(5);
  auto node = rclcpp::Node::make_shared(node_name);

  // data directory
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

  // Parameters
  const unsigned run_id = node->declare_parameter<int>("run_id", 0);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

  // get the temporal pose graph
  auto evaluator =
      std::make_shared<tactic::TemporalEvaluator<tactic::GraphBase>>(*graph);
  auto subgraph = graph->getSubgraph(tactic::VertexId(run_id, 0), evaluator);

  //
  GraphAnnotator annotator(node, *subgraph, tactic::VertexId(run_id, 0));

  //
  rclcpp::spin(node);
  rclcpp::shutdown();

  //
  CLOG(WARNING, "test") << "Saving pose graph.";
  graph->save();
}