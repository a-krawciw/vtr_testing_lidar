#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/pipeline.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"

#include "vtr_lidar/data_types/pointscan.hpp"
#include "vtr_testing_honeycomb/utils.hpp"
#include <boost/algorithm/string.hpp>


namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::testing;


int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  rclcpp::init(argc, argv);
  const std::string node_name = "graph_fun_" + random_string(5);
  auto node = rclcpp::Node::make_shared(node_name);

  // teach sequence ros2bag
  const auto teach_dir_str =
      node->declare_parameter<std::string>("teach_bag", "./tmp");
  fs::path odo_dir{utils::expand_user(utils::expand_env(teach_dir_str))};

  // repeat sequence ros2bag
  const auto rep_dir_str =
      node->declare_parameter<std::string>("repeat_bag", "./tmp");
  fs::path rep_dir{utils::expand_user(utils::expand_env(rep_dir_str))};

  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "./tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Number of frames to include
  const auto num_frames = node->declare_parameter<int>("num_frames", 100000);

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

  CLOG(WARNING, "test") << "Teach Directory: " << odo_dir.string();
  CLOG(WARNING, "test") << "Repeat Directory: " << rep_dir.string();
  CLOG(WARNING, "test") << "Output Directory: " << data_dir.string();

  std::vector<std::string> parts;
  boost::split(parts, teach_dir_str, boost::is_any_of("/"));
  auto stem = parts.back();
  boost::replace_all(stem, "-", "_");
  CLOG(WARNING, "test") << "Publishing status to topic: "
                        << (stem + "_lidar_odometry");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      stem + "_lidar_odometry", 1);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);
  auto eval = std::make_shared<TemporalEvaluator<Graph>>(*graph);

  auto bfs_iter = graph->begin(VertexId(1,0), 0, eval);
  for(; bfs_iter != graph->end(); ++bfs_iter){
    auto vertex_ptr = bfs_iter->v();
    CLOG(DEBUG, "test") << *vertex_ptr;
    auto map_ = vertex_ptr->retrieve<lidar::PointScan<lidar::PointWithInfo>>("filtered_point_cloud", "vtr_lidar_msgs/msg/PointScan");
    auto locked_msg = map_->sharedLocked();
    auto point_map_mess = locked_msg.get().getData();
    CLOG(DEBUG, "test") << point_map_mess.point_cloud().size();
  }


  CLOG(DEBUG, "test") << "Vertices:" << graph->numberOfVertices() << " Edges:" << graph->numberOfEdges();

  CLOG(WARNING, "test") << "Saving pose graph.";
  graph->save();

//
//-----------Repeat Pass-----------------
//

  rclcpp::shutdown();

  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";

}