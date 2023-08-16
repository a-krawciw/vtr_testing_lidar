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

#include "vtr_testing_honeycomb/utils.hpp"
#include <boost/algorithm/string.hpp>
#include <regex>

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
  const std::string node_name = "change_detection_" + random_string(5);
  auto node = rclcpp::Node::make_shared(node_name);

  // teach sequence ros2bag
  const auto teach_dir_str =
      node->declare_parameter<std::string>("sequence_dir", "./tmp");
  fs::path seq_dir{utils::expand_user(utils::expand_env(teach_dir_str))};

  fs::path odo_dir;
  std::vector<fs::path> repeat_dirs;

  std::regex teach_regex(".*_teach"); 
  std::regex repeat_regex(".*_repeat"); 

  for (const auto & file: fs::directory_iterator(seq_dir)) {
    if (file.is_directory() && std::regex_search(file.path().string(), teach_regex))
      odo_dir = file;
    else if (file.is_directory() && std::regex_search(file.path().string(), repeat_regex))
      repeat_dirs.push_back(file);
  }

  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "./tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  const bool reversed_path = node->declare_parameter<bool>("reversed_path", false);

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

  

  CLOG(INFO, "test") << "Teach Directory: " << odo_dir.string();
  for (auto & file : repeat_dirs)
    CLOG(INFO, "test") << "Repeat Directory: " << file.string();
  CLOG(INFO, "test") << "Output Directory: " << data_dir.string();

  std::vector<std::string> parts;
  boost::split(parts, teach_dir_str, boost::is_any_of("/"));
  auto stem = parts.back();
  boost::replace_all(stem, "-", "_");
  CLOG(WARNING, "test") << "Publishing status to topic: "
                        << (stem + "_lidar_odometry");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(
      stem + "_lidar_odometry", 1);

  // Pipeline
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node);
  auto pipeline = pipeline_factory->get("pipeline");
  auto pipeline_output = pipeline->createOutputCache();
  // some modules require node for visualization
  pipeline_output->node = node;

  // Tactic Callback
  auto callback = std::make_shared<RvizTacticCallback>(node);


  const std::string graph_name = node->declare_parameter<std::string>("name", "graph");
  float map_voxel_size = 0;
  node->get_parameter<float>("odometry.mapping.map_voxel_size", map_voxel_size);
  CLOG(INFO, "test") << "Frame voxel size is " << map_voxel_size;

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / graph_name).string(), false);

  // Tactic
  auto tactic =
      std::make_shared<Tactic>(Tactic::Config::fromROS(node), pipeline,
                               pipeline_output, graph, callback);
  tactic->setPipeline(PipelineMode::TeachBranch);
  tactic->addRun();

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "lidar";

  auto points_topic = node->declare_parameter<std::string>("lidar_topic", "/points");

  /// robot lidar transformation is hard-coded - check measurements.
  Eigen::Matrix4d T_lidar_robot_mat;
  T_lidar_robot_mat << 1, 0, 0, -0.025, 0, -1, 0, -0.002, 0, 0, -1, 0.87918, 0, 0, 0, 1;
  EdgeTransform T_lidar_robot(T_lidar_robot_mat);
  T_lidar_robot.setZeroCovariance();
  CLOG(WARNING, "test") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_lidar_robot;

  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg =
      tf2::eigenToTransform(Eigen::Affine3d(T_lidar_robot.inverse().matrix()));
  msg.header.frame_id = robot_frame;
  msg.child_frame_id = lidar_frame;
  tf_sbc->sendTransform(msg);

  const auto clock_publisher =
      node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // Load dataset
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = odo_dir.string();
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;  // default
  storage_options.max_cache_size = 0;    // default
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(points_topic);
  filter.topics.push_back("/" + points_topic);

  rosbag2_cpp::Reader reader;
  reader.open(storage_options, converter_options);
  reader.set_filter(filter);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  // thread handling variables
  TestControl test_control(node);
  storage::Timestamp starttime = 0;

  // main loop
  int frame = 0;
  while (reader.has_next()) {
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    if (num_frames > 0 && frame >= num_frames) break;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(test_control.delay()));

    ///
    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage msg(*bag_message->serialized_data);
    auto points = std::make_shared<sensor_msgs::msg::PointCloud2>();
    serializer.deserialize_message(&msg, points.get());
    storage::Timestamp timestamp =
        points->header.stamp.sec * 1e9 + points->header.stamp.nanosec;

    if (starttime < 1)
      starttime = timestamp;

    CLOG(WARNING, "test") << "Loading lidar frame " << frame
                          << " with timestamp " << timestamp << "(" << (timestamp - starttime) / 1e9  << "s)";

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(timestamp);
    clock_publisher->publish(time_msg);

    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<lidar::LidarQueryCache>();

    // some modules require node for visualization
    query_data->node = node;

    // set timestamp
    query_data->stamp.emplace(timestamp);

    // make up some environment info (not important)
    tactic::EnvInfo env_info;
    env_info.terrain_type = 0;
    query_data->env_info.emplace(env_info);

    // set lidar frame
    query_data->pointcloud_msg = points;

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_lidar_robot);

    // execute the pipeline
    tactic->input(query_data);

    std_msgs::msg::String status_msg;
    status_msg.data = "Finished processing lidar frame " +
                      std::to_string(frame) + " with timestamp " +
                      std::to_string(timestamp);
    status_publisher->publish(status_msg);

    ++frame;
  }

  tactic->finishRun();
  CLOG(WARNING, "test") << "Saving pose graph.";
  graph->save();

//
//-----------Repeat Pass-----------------
//


  // Get the path that we should repeat
  VertexId::Vector sequence;
  sequence.reserve(graph->numberOfVertices());
  CLOG(WARNING, "test") << "Total number of vertices: "
                        << graph->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  using LocEvaluator = tactic::PrivilegedEvaluator<tactic::GraphBase>;
  auto evaluator = std::make_shared<LocEvaluator>(*graph);
  auto privileged_path = graph->getSubgraph(0ul, evaluator);
  std::stringstream ss;
  ss << "Teach vertices: ";
  double total_distance = 0.0;
  for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
       ++it) {
    ss << it->v()->id() << " ";
    total_distance += it->T().r_ab_inb().norm();
    if (reversed_path) {
      if (sequence.size() <= 706ul)
        sequence.insert(sequence.begin(), it->v()->id());
    } else {
      sequence.push_back(it->v()->id());
    }
  }
  
  ss << "Total distance: " << total_distance;
  CLOG(WARNING, "test") << ss.str();

for (auto& repeat_dir : repeat_dirs) {
  rosbag2_cpp::Reader reader2;

  try {
    // Load dataset
    storage_options.uri = repeat_dir.string();

    reader2.open(storage_options, converter_options);
    reader2.set_filter(filter);
  } catch (...) {
    CLOG(ERROR, "test") << "Repeat bag invalid";
    return 10;
  }

  tactic->setPipeline(PipelineMode::RepeatFollow);
  tactic->addRun();


  EdgeTransform T_loc_odo_init(true);
  T_loc_odo_init.setCovariance(Eigen::Matrix<double, 6, 6>::Identity());
  tactic->setPath(sequence, /* trunk sid */ 0, T_loc_odo_init, true);


  // main loop
  frame = 0;
  while (reader2.has_next()) {
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(test_control.delay()));

    ///
    auto bag_message = reader2.read_next();
    rclcpp::SerializedMessage msg(*bag_message->serialized_data);
    auto points = std::make_shared<sensor_msgs::msg::PointCloud2>();
    serializer.deserialize_message(&msg, points.get());
    storage::Timestamp timestamp =
        points->header.stamp.sec * 1e9 + points->header.stamp.nanosec;

    CLOG(WARNING, "test") << "Loading lidar frame " << frame
                          << " with timestamp " << timestamp;

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(timestamp);
    clock_publisher->publish(time_msg);

    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<lidar::LidarQueryCache>();

    // some modules require node for visualization
    query_data->node = node;

    // set timestamp
    query_data->stamp.emplace(timestamp);

    // make up some environment info (not important)
    tactic::EnvInfo env_info;
    env_info.terrain_type = 0;
    query_data->env_info.emplace(env_info);

    // set lidar frame
    query_data->pointcloud_msg = points;

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_lidar_robot);

    std_msgs::msg::String status_msg;

    try{ 
      // execute the pipeline
      tactic->input(query_data);
      status_msg.data = "Finished processing lidar frame " +
                        std::to_string(frame) + " with timestamp " +
                        std::to_string(timestamp);
    } catch(std::runtime_error& e) {
      CLOG(ERROR, "test") << "Pipeline failed for frame." << std::to_string(frame) << " Error: " << e.what();

      status_msg.data = "Failed processing lidar frame " +
                        std::to_string(frame) + " with timestamp " +
                        std::to_string(timestamp);
      
      rclcpp::shutdown();
      break;
    }
    status_publisher->publish(status_msg);

    if (tactic->routeCompleted()) {
      CLOG(WARNING, "test") << "Route completed!";
      break;
    }

    ++frame;
  }
    auto plock = tactic->lockPipeline();
    tactic->finishRun();
    pipeline_output->chain->reset();
    CLOG(WARNING, "test") << "Saving pose graph.";
}
  CLOG(ERROR, "test") << "Reached the end";


  rclcpp::shutdown();

  tactic.reset();
  CLOG(ERROR, "test") << "Reached the end";

  callback.reset();
  pipeline.reset();
  pipeline_factory.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";

}