/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_vtr_plugins/tools/publish_select/publish_selection_tool.hpp"

#include <QKeyEvent>  // NOLINT cpplint cannot handle include order

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace rviz_vtr_plugins {
namespace tools {

PublishSelectionTool::PublishSelectionTool() = default;

PublishSelectionTool::~PublishSelectionTool() = default;

void PublishSelectionTool::onInitialize() {
  Parent::onInitialize();
  //
  rclcpp::Node::SharedPtr raw_node =
      context_->getRosNodeAbstraction().lock()->get_raw_node();
  pc_publisher_ =
      raw_node->template create_publisher<sensor_msgs::msg::PointCloud2>(
          "/selected_points", qos_profile_);
  cmd_publisher_ = raw_node->template create_publisher<std_msgs::msg::Int32>(
      "/selection_command", qos_profile_);
  clock_ = raw_node->get_clock();

  // point cloud
  // selected_points_.header.frame_id = context_->getFixedFrame().toStdString();
  selected_points_.header.frame_id = "unknown";
  selected_points_.header.stamp = clock_->now();
  selected_points_.height = 1;
  selected_points_.point_step = 3 * 4;
  selected_points_.is_dense = false;
  selected_points_.is_bigendian = false;
  selected_points_.fields.resize(3);

  selected_points_.fields[0].name = "x";
  selected_points_.fields[0].offset = 0;
  selected_points_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  selected_points_.fields[0].count = 1;

  selected_points_.fields[1].name = "y";
  selected_points_.fields[1].offset = 4;
  selected_points_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  selected_points_.fields[1].count = 1;

  selected_points_.fields[2].name = "z";
  selected_points_.fields[2].offset = 8;
  selected_points_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  selected_points_.fields[2].count = 1;
}

void PublishSelectionTool::activate() {
  Parent::activate();
  selecting_ = false;
}

int PublishSelectionTool::processMouseEvent(
    rviz_common::ViewportMouseEvent &event) {
  //
  int flags = Parent::processMouseEvent(event);

  //
  if (event.alt()) {
    selecting_ = false;
  } else {
    if (event.leftDown()) {
      selecting_ = true;
    }
  }

  if (selecting_) {
    if (event.leftUp()) {
      // process selected area
      processSelectedArea();

      //
      selecting_ = false;
    }
  }

  //
  return flags;
}

int PublishSelectionTool::processKeyEvent(QKeyEvent *event,
                                          rviz_common::RenderPanel *panel) {
  //
  int render = Parent::processKeyEvent(event, panel);

  //
  if (event->key() == Qt::Key_P) {
    RVIZ_COMMON_LOG_INFO("Publish the selected points.");
    pc_publisher_->publish(selected_points_);
  } else if (event->key() == Qt::Key_C) {
    RVIZ_COMMON_LOG_INFO("Should clear the point cloud.");
    command_.data = -2;  /// cancel the selection
    cmd_publisher_->publish(command_);
  } else if (event->key() == Qt::Key_N) {
    RVIZ_COMMON_LOG_INFO("Should proceed to the next point cloud.");
    command_.data = -1;  /// proceed to the next point cloud
    cmd_publisher_->publish(command_);
  } else if (event->key() == Qt::Key_0) {
    RVIZ_COMMON_LOG_INFO("Switch to annotate points with type 0");
    command_.data = 0;
    cmd_publisher_->publish(command_);
  } else if (event->key() == Qt::Key_1) {
    RVIZ_COMMON_LOG_INFO("Switch to annotate points with type 1");
    command_.data = 1;
    cmd_publisher_->publish(command_);
  } else if (event->key() == Qt::Key_2) {
    RVIZ_COMMON_LOG_INFO("Switch to annotate points with type 2");
    command_.data = 2;
    cmd_publisher_->publish(command_);
  }

  //
  return render;
}

void PublishSelectionTool::processSelectedArea() {
  //
  auto selection_manager = context_->getSelectionManager();
  auto *model = selection_manager->getPropertyModel();

  //
  const int num_rows = model->rowCount();
  std::vector<int> point_rows;
  for (int i = 0; i < num_rows; ++i) {
    const auto *prop = model->getProp(model->index(i, 0));
    if (prop->getName().startsWith("Point")) point_rows.emplace_back(i);
  }
  RVIZ_COMMON_LOG_INFO_STREAM(
      "Number of selected points: " << point_rows.size());

  selected_points_.data.resize(point_rows.size() * selected_points_.point_step);

  for (int i = 0; i < (int)point_rows.size(); ++i) {
    const auto &row = point_rows[i];
    const auto *prop = model->getProp(model->index(row, 0));
    //
    auto *subprop = dynamic_cast<rviz_common::properties::VectorProperty *>(
        prop->childAt(0));
    if (!subprop) throw std::runtime_error("selected point property is null");

    auto point_data = subprop->getVector();
    uint8_t *data_pointer =
        &selected_points_.data[0] + i * selected_points_.point_step;
    *(float *)data_pointer = point_data.x;
    data_pointer += 4;
    *(float *)data_pointer = point_data.y;
    data_pointer += 4;
    *(float *)data_pointer = point_data.z;
  }

  selected_points_.row_step = point_rows.size() * selected_points_.point_step;
  selected_points_.width = point_rows.size();
  selected_points_.header.stamp = clock_->now();
}

}  // namespace tools
}  // namespace rviz_vtr_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_vtr_plugins::tools::PublishSelectionTool,
                       rviz_common::Tool)
