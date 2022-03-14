#pragma once

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rviz_default_plugins/tools/select/selection_tool.hpp"

namespace rviz_vtr_plugins {
namespace tools {

class PublishSelectionTool : public rviz_default_plugins::tools::SelectionTool {
 public:
  using Parent = rviz_default_plugins::tools::SelectionTool;

  PublishSelectionTool();
  ~PublishSelectionTool() override;

  void onInitialize() override;

  void activate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;
  int processKeyEvent(QKeyEvent *event,
                      rviz_common::RenderPanel *panel) override;

 protected:
  void processSelectedArea();

 protected:
  rclcpp::QoS qos_profile_{5};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

 private:
  bool selecting_ = false;
  sensor_msgs::msg::PointCloud2 selected_points_;
};

}  // namespace tools
}  // namespace rviz_vtr_plugins
