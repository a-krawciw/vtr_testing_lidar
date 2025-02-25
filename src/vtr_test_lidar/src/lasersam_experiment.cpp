#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/pipeline.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"
#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"
#include "vtr_lidar/filters/voxel_downsample.hpp"

#include "vtr_testing_honeycomb/utils.hpp"
#include <boost/algorithm/string.hpp>
#include <regex>

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::testing;
using namespace vtr::lidar;




int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);


  rclcpp::init(argc, argv);
  const std::string node_name = "change_detection_" + random_string(5);
  auto node = rclcpp::Node::make_shared(node_name);



  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "./tmp");
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

  const std::string graph_name = node->declare_parameter<std::string>("name", "graph");
  const float nn_voxel_size = node->declare_parameter<float>("nn_voxel_size", 0.05);
  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / graph_name).string(), true);
  CLOG(DEBUG, "test") << "Hello 1";


  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node);
  auto pipeline = pipeline_factory->get("pipeline");
  auto pipeline_output = pipeline->createOutputCache();
    auto tactic =
      std::make_shared<Tactic>(Tactic::Config::fromROS(node), pipeline,
                               pipeline_output, graph, std::make_shared<RvizTacticCallback>(node));  // some modules require node for visualization
  pipeline_output->node = node;
    auto &chain = pipeline_output->chain;


  //const ModuleFactory::Ptr mod_fac = std::make_shared<ModuleFactory>();
  lidar::ChangeDetectionModuleV3 baseline{lidar::ChangeDetectionModuleV3::Config::fromROS(node, "localization.change_detection_sync")};
  lidar::SegmentAnythingModule lasersam{lidar::SegmentAnythingModule::Config::fromROS(node, "localization.lasersam")};
  lidar::CDTestModule save_baseline{lidar::CDTestModule::Config::fromROS(node, "localization.save_nearest")};
  lidar::CDTestModule save_lasersam{lidar::CDTestModule::Config::fromROS(node, "localization.save_lasersam")};
  lidar::PixelDiffCDModule pixel_diff{lidar::PixelDiffCDModule::Config::fromROS(node, "localization.pixel_diff")};
  lidar::RangeChangeNetModule range_change{lidar::RangeChangeNetModule::Config::fromROS(node, "localization.range_change")};
  lidar::CDTestModule save_range{lidar::CDTestModule::Config::fromROS(node, "localization.save_range_change")};

  CLOG(DEBUG, "test") << "Hello";
  const auto &vertices_in_use = node->declare_parameter<std::vector<std::string>>("vertices_in_use", std::vector<std::string>{});
  std::vector<tactic::VertexId> vertex_ids;

  for(const auto &v_str : vertices_in_use) {
    vertex_ids.push_back(tactic::VertexId(std::stoul(v_str)));
  }

  const auto &teach_vertices_in_use = node->declare_parameter<std::vector<std::string>>("teach_vs_in_use", std::vector<std::string>{});
  std::vector<tactic::VertexId> teach_vertex_ids;

  for(const auto &v_str : teach_vertices_in_use) {
    teach_vertex_ids.push_back(tactic::VertexId(std::stoul(v_str)));
  }

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "lidar";

  auto points_topic = node->declare_parameter<std::string>("lidar_topic", "/points");

  /// robot lidar transformation is hard-coded - check measurements.
  Eigen::Matrix4d T_lidar_robot_mat;
  //T_lidar_robot_mat //<< 1, 0, 0, -0.025, 0, -1, 0, -0.002, 0, 0, -1, 0.87918, 0, 0, 0, 1;
  T_lidar_robot_mat << -1, 0,  0,  0.025,
        0, 1, 0, 0.00200248,
        0, 0, -1, 0.843,
        0, 0, 0, 1;
  EdgeTransform T_lidar_robot(T_lidar_robot_mat);
  T_lidar_robot.setZeroCovariance();
  CLOG(WARNING, "test") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_lidar_robot;


// Get the path that we should repeat
  VertexId::Vector sequence;
  sequence.reserve(graph->numberOfVertices());
  CLOG(WARNING, "test") << "Total number of vertices: "
                        << graph->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  using LocEvaluator = tactic::PrivilegedEvaluator<tactic::GraphBase>;
  auto evaluator = std::make_shared<LocEvaluator>(*graph);
  auto privileged_path = graph->getSubgraph(0ul, evaluator);
  for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
       ++it) {
    sequence.push_back(it->v()->id());
  }
  
  chain->setSequence(sequence);


  
//
//-----------Repeat Pass-----------------
//



//   // main loop
unsigned i = 0;
for (const auto &v_id : vertex_ids) {

    auto vertex = graph->at(v_id);

    CLOG(WARNING, "test") << "Loading lidar frame at vertex" << v_id
                          << " with timestamp " << vertex->vertexTime();


    // Convert message to qdata format and store into qdata
    auto qdata = std::make_shared<lidar::LidarQueryCache>();

    // some modules require node for visualization
    qdata->node = node;


    // set timestamp
    qdata->stamp.emplace(vertex->vertexTime());
    qdata->vid_odo.emplace(v_id);


    qdata->T_r_v_odo.emplace(EdgeTransform(true));


    auto raw_scan = [&] {
        auto locked_nn_pc_msg = vertex->retrieve<PointScan<PointWithInfo>>(
                "raw_point_cloud", "vtr_lidar_msgs/msg/PointScan");

        if (locked_nn_pc_msg != nullptr) {
            auto locked_msg = locked_nn_pc_msg->sharedLocked();
            return locked_msg.get().getDataPtr();
        }
        CLOG(WARNING, "lidar.perspective") << "Could not load raw view from repeat";
        return std::make_shared<PointScan<PointWithInfo>>();
    }();
    qdata->raw_point_cloud.emplace(raw_scan->point_cloud());

    auto nn_point_cloud = pcl::PointCloud<PointWithInfo>(*qdata->raw_point_cloud);
    voxelDownsample(nn_point_cloud, nn_voxel_size);
    qdata->nn_point_cloud.emplace(nn_point_cloud);

  

    VertexId map_vid = teach_vertex_ids.at(i);
    auto map_vertex = graph->at(map_vid);

    auto point_map_ptr = [&] {
      using PointMapPointerLM = storage::LockableMessage<PointMapPointer>;
    auto submap_ptr_msg = map_vertex->retrieve<PointMapPointer>(
        "pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
        
      if (submap_ptr_msg != nullptr) {
        auto locked_msg = submap_ptr_msg->sharedLocked();
        return locked_msg.get().getDataPtr();
      }
      CLOG(WARNING, "lidar.perspective") << "Could not load pointmap pointer from teach";
      return std::make_shared<PointMapPointer>();
    }();

    CLOG(DEBUG, "test") << "Ptr points to " << point_map_ptr->map_vid;


    auto map_scan = [&] {
        auto locked_nn_pc_msg = graph->at(point_map_ptr->map_vid)->retrieve<PointMap<PointWithInfo>>(
                "pointmap", "vtr_lidar_msgs/msg/PointMap");

        if (locked_nn_pc_msg != nullptr) {
            auto locked_msg = locked_nn_pc_msg->sharedLocked();
            return locked_msg.get().getDataPtr();
        }
        CLOG(WARNING, "lidar.perspective") << "Could not load map view from teach";
        return std::make_shared<PointMap<PointWithInfo>>(0.3);
    }();
    qdata->submap_loc.emplace(*map_scan);
    qdata->T_v_m_loc.emplace(point_map_ptr->T_v_this_map *
                          qdata->submap_loc->T_vertex_this());


    EdgeTransform T_r_v_loc{true};
    auto connected = graph->dijkstraSearch(v_id, map_vid);
    T_r_v_loc = pose_graph::eval::ComposeTfAccumulator(connected->beginDfs(v_id), connected->end(), T_r_v_loc);

    chain->resetTrunk(map_vid.minorId());
    chain->setPetiole(v_id);
    // chain_.updateBranchToTwigTransform(live_id, map_id, map_sid,
    //                                    T_petiole_trunk, true, false);
    // chain->updatePetioleToLeafTransform(EdgeTransform(true), true, false);


    qdata->sid_loc.emplace(map_vid.minorId());
    CLOG(DEBUG, "test") << "Map id" << map_vid;

    qdata->vid_loc.emplace(map_vid);
    qdata->T_r_v_loc.emplace(T_r_v_loc);

    CLOG(DEBUG, "test") << "Localization tf " << *qdata->T_r_v_loc;

    

    //sid_loc
    //vid_loc from pointmap_ptr
    //T_r_v_loc
    //T_v_m_loc

    qdata->run_cd.emplace(true);
    qdata->vertex_test_result.emplace(VertexTestResult::CREATE_VERTEX);
    // set lidar frame check
    // set raw points check
    // set filtered points
    // set vid_odo to be the target vertex
    // 
    //qdata->pointcloud_msg = points;

    // fill in the vehicle to sensor transform and frame name
    qdata->T_s_r.emplace(T_lidar_robot);


    try{ 
      // execute the pipeline
      auto exec = std::make_shared<tactic::TaskExecutor>(pipeline_output, graph, 1);
      baseline.run(*qdata, *pipeline_output, graph, exec);
      save_baseline.run(*qdata, *pipeline_output, graph, exec);
      lasersam.run(*qdata, *pipeline_output, graph, exec);
      save_lasersam.run(*qdata, *pipeline_output, graph, exec);
      pixel_diff.run(*qdata, *pipeline_output, graph, exec);
      range_change.run(*qdata, *pipeline_output, graph, exec);
      save_range.run(*qdata, *pipeline_output, graph, exec);
    } catch(std::runtime_error& e) {
      CLOG(ERROR, "test") << "Pipeline failed for frame." << v_id << " Error: " << e.what();
      
      rclcpp::shutdown();
      break;
    }

  ++i;

}

  rclcpp::shutdown();

  CLOG(ERROR, "test") << "Reached the end";

  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";

}