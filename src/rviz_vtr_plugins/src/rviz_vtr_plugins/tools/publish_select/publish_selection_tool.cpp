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

#include "rviz_common/display_context.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace rviz_vtr_plugins {
namespace tools {

PublishSelectionTool::PublishSelectionTool() {
  // rclcpp::Node::SharedPtr raw_node =
  //     context_->getRosNodeAbstraction().lock()->get_raw_node();
  // publisher_ =
  //     raw_node->template create_publisher<std_msgs::msg::Int32MultiArray>(
  //         "/selected_points", qos_profile_);
  // clock_ = raw_node->get_clock();
}

PublishSelectionTool::~PublishSelectionTool() = default;

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

void PublishSelectionTool::processSelectedArea() {
  //
  RVIZ_COMMON_LOG_INFO("This should be logged.");
}

}  // namespace tools
}  // namespace rviz_vtr_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_vtr_plugins::tools::PublishSelectionTool,
                       rviz_common::Tool)
