#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"

#include <boost/algorithm/string.hpp>


namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;




int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  const std::string node_name = "trim_";
  auto node = rclcpp::Node::make_shared(node_name);



  // Output directory
  const auto graph_dir_str =
      node->declare_parameter<std::string>("graph_dir", "./tmp");
  fs::path graph_dir{utils::expand_user(utils::expand_env(graph_dir_str))};

  auto graph = tactic::Graph::MakeShared((graph_dir / "graph").string(), true);
  std::vector<VertexId> targets = {VertexId(0, 0),
  VertexId(5, 252),
  VertexId(12, 5),
  VertexId(0, 52),
  VertexId(0, 0),
  VertexId(14, 139)
  };
  auto smaller_graph = graph->getSubgraph(targets);

  auto graph_small = tactic::Graph::MakeShared()
  smaller_graph->save();
  rclcpp::shutdown();
}