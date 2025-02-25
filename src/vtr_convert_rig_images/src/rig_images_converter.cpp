#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include <sensor_msgs/msg/image.hpp>

#include "vtr_common/utils/filesystem.hpp"

#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/msg/rig_image_calib.hpp>
#include <vtr_messages/msg/image.hpp>

#include<iostream>


namespace fs = std::filesystem;
using namespace vtr::common;
using Image = sensor_msgs::msg::Image;
using VtrImage = vtr_messages::msg::Image;
using VtrImagePtr = std::shared_ptr<VtrImage>;


int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  const std::string node_name = "conversion";
  auto node = rclcpp::Node::make_shared(node_name);

  // teach sequence ros2bag
  const auto old_bag_str =
      node->declare_parameter<std::string>("old_bag", "./tmp");
  fs::path old_bag{utils::expand_user(utils::expand_env(old_bag_str))};

  // repeat sequence ros2bag
  const auto new_bag_str =
      node->declare_parameter<std::string>("new_bag", "./tmp");
  fs::path new_bag{utils::expand_user(utils::expand_env(new_bag_str))};


  auto topic = node->declare_parameter<std::string>("images_topic", "/images");

  // Load dataset
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = old_bag.string();
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;  // default
  storage_options.max_cache_size = 0;    // default
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(topic);
  filter.topics.push_back("/" + topic);

  rosbag2_cpp::Reader reader;
  reader.open(storage_options, converter_options);
  reader.set_filter(filter);

  rclcpp::Serialization<vtr_messages::msg::RigImageCalib> serializer;


  rosbag2_storage::TopicMetadata left_topic;
  left_topic.name = "/xb3/RGB/left_image";
  left_topic.type = "sensor_msgs/msg/Image";
  left_topic.serialization_format = "cdr";


  rosbag2_storage::TopicMetadata right_topic;
  right_topic.name = "/xb3/RGB/right_image";
  right_topic.type = "sensor_msgs/msg/Image";
  right_topic.serialization_format = "cdr";

  auto writer = std::make_unique<rosbag2_cpp::Writer>();

  writer->open(new_bag.string());

  writer->create_topic(left_topic);

  writer->create_topic(right_topic);

  // main loop
  int frame = 0;
  while (reader.has_next()) {
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);

    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage msg(*bag_message->serialized_data);
    auto calib_images = std::make_shared<vtr_messages::msg::RigImageCalib>();
    serializer.deserialize_message(&msg, calib_images.get());

    auto &rig_images = calib_images->rig_images;

    auto left_image = std::make_shared<Image>();
    auto right_image = std::make_shared<Image>();

    for (auto &channel : rig_images.channels) {
      if (channel.cameras.size() == 2) {
        VtrImage &left_vtr = channel.cameras.at(0);
        VtrImage &right_vtr = channel.cameras.at(1);

        left_image->header.stamp = rclcpp::Time(left_vtr.stamp.nanoseconds_since_epoch);
        left_image->height = left_vtr.height;
        left_image->width = left_vtr.width;
        left_image->encoding = left_vtr.encoding;
        left_image->is_bigendian = left_vtr.is_bigendian;
        left_image->step = left_vtr.step * 3;
        left_image->data = left_vtr.data;

        right_image->header.stamp = rclcpp::Time(right_vtr.stamp.nanoseconds_since_epoch);
        right_image->height = right_vtr.height;
        right_image->width = right_vtr.width;
        right_image->encoding = right_vtr.encoding;
        right_image->is_bigendian = right_vtr.is_bigendian;
        right_image->step = right_vtr.step * 3;
        right_image->data = right_vtr.data;

        writer->write(*left_image, left_topic.name, left_image->header.stamp);
        writer->write(*right_image, right_topic.name, right_image->header.stamp);

      } else {
        std::cerr << "Error length was " << channel.cameras.size();
        rclcpp::shutdown();
        return -1;
      }
    }
  



    ++frame;
  }

  rclcpp::shutdown();
  return 0;
}