#pragma once

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "rviz_default_plugins/tools/select/selection_tool.hpp"

namespace rviz_vtr_plugins {
namespace tools {

class PublishSelectionTool : public rviz_default_plugins::tools::SelectionTool {
 public:
  using Parent = rviz_default_plugins::tools::SelectionTool;

  PublishSelectionTool();
  ~PublishSelectionTool() override;

  void activate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;
  // int processKeyEvent(QKeyEvent *event,
  //                     rviz_common::RenderPanel *panel) override;

 protected:
  void processSelectedArea();

 protected:
  rclcpp::QoS qos_profile_{5};
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

 private:
  bool selecting_ = false;
};

}  // namespace tools
}  // namespace rviz_vtr_plugins
