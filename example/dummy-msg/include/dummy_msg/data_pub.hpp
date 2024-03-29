// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_DUMMY_MSG_DATA_PUB_HPP_
#define INCLUDE_DUMMY_MSG_DATA_PUB_HPP_

#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

std::vector<uint8_t> floatArrayToByteArray(const std::vector<float>& floatArray);

class PublisherNode : public rclcpp::Node {
 public:
  PublisherNode();

 private:
  void publish_message();
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // INCLUDE_DUMMY_MSG_DATA_PUB_HPP_

