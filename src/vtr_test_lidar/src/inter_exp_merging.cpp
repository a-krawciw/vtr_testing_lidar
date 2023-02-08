#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
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

class ExpFilter : public vtr::pose_graph::eval::mask::BaseEval {
 public:
  PTR_TYPEDEFS(ExpFilter);

  ExpFilter(const std::unordered_set<uint32_t> &majorids)
      : majorids_(majorids) {}

 protected:
  vtr::pose_graph::eval::mask::ReturnType computeEdge(const EdgeId &) override {
    return true;
  }

  vtr::pose_graph::eval::mask::ReturnType computeVertex(
      const VertexId &id) override {
    // We can't tell direction if the majors are different
    if (majorids_.count(id.majorId())) return true;
    // Nope!
    return false;
  }

 private:
  const std::unordered_set<uint32_t> majorids_;
};

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  rclcpp::init(argc, argv);
  const std::string node_name = "inter_exp_merging_" + random_string(5);
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

  // Parameters
  const unsigned run_id = node->declare_parameter<int>("run_id", 1);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);
  try {
    graph->at(tactic::VertexId(run_id, 0));
  } catch (const std::range_error &) {
    CLOG(ERROR, "test") << "Specified run: " << run_id << " does not exist.";
    return 1;
  }

  // Module
  auto module_factory = std::make_shared<ROSModuleFactory>(node);
  auto module = module_factory->get("odometry.inter_exp_merging");

  // thread handling variables
  TestControl test_control(node);

  // clang-format off

  /// Find update sets
  auto evaluator = std::make_shared<tactic::TemporalEvaluator<tactic::GraphBase>>(*graph);
  auto exp_filter = std::make_shared<ExpFilter>(std::unordered_set<uint32_t>{run_id, 0});

  auto priv_subgraph = graph->getSubgraph(tactic::VertexId(0, 0), evaluator);
  auto curr_subgraph = graph->getSubgraph(tactic::VertexId(run_id, 0), evaluator);
  auto filtered_subgraph = graph->getSubgraph(tactic::VertexId(0, 0), exp_filter);

  std::vector<std::tuple<tactic::VertexId, tactic::VertexId, tactic::EdgeTransform>> update_tuples;
  std::unordered_map<tactic::VertexId, int> update_map_counts;

  for (auto it = curr_subgraph->begin(tactic::VertexId(run_id, 0)); it != curr_subgraph->end(); ++it) {
    const auto curr_vertex = it->v();
    const auto curr_vid = it->v()->id();

    // check if we have a map for this vertex
    {
      const auto msg = curr_vertex->retrieve<PointMapPointer>("pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
      const auto pointmap_ptr = msg->sharedLocked().get().getData();
      if (pointmap_ptr.map_vid != curr_vid)
        continue;
    }

    //
    pose_graph::PoseCache<GraphBase> pose_cache(filtered_subgraph, curr_vid);

    for (auto it2 = priv_subgraph->begin(tactic::VertexId(0, 0)); it2 != priv_subgraph->end(); ++ it2) {
      const auto priv_vertex = it2->v();
      const auto priv_vid = it2->v()->id();

      // check if we have a map for this vertex
      {
        const auto msg = priv_vertex->retrieve<PointMapPointer>("pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer");
        const auto pointmap_ptr = msg->sharedLocked().get().getData();
        if (pointmap_ptr.map_vid != priv_vid) continue;
      }

      // check if this vertex is close enough
      auto T_curr_priv = pose_cache.T_root_query(priv_vid);
      /// \note tune this!!
      if (T_curr_priv.r_ab_inb().norm() > 1.2) continue;

      // add to queue
      update_tuples.emplace_back(priv_vid, curr_vid, T_curr_priv.inverse());
      update_map_counts.try_emplace(priv_vid, 0);
      update_map_counts.try_emplace(curr_vid, 0);
      update_map_counts.at(priv_vid)++;
      update_map_counts.at(curr_vid)++;

      CLOG(WARNING, "test") << "Found update pair: " << priv_vid << " " << curr_vid << " " << T_curr_priv.r_ab_inb().norm();
    }
  }

  for (const auto [priv_vid, curr_vid, T_priv_curr] : update_tuples) {
    /// test control
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(test_control.delay()));

    CLOG(WARNING, "test") << "Update pair: " << priv_vid << " " << curr_vid;

    /// caches
    lidar::LidarQueryCache qdata;
    lidar::LidarOutputCache output;

    qdata.node = node;
    qdata.inter_exp_merging_async.emplace(priv_vid, curr_vid, T_priv_curr);

    module->runAsync(qdata, output, graph, nullptr, {}, {});

    // memory management
    update_map_counts.at(curr_vid)--;
    if (update_map_counts.at(curr_vid) == 0) {
      graph->at(curr_vid)->unload();
      // CLOG(WARNING, "test") << "Unloaded: " << curr_vid;
    }
    update_map_counts.at(priv_vid)--;
    if (update_map_counts.at(priv_vid) == 0) {
      graph->at(priv_vid)->unload();
      // CLOG(WARNING, "test") << "Unloaded: " << priv_vid;
    }
  }

  rclcpp::shutdown();

  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";
}