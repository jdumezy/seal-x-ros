// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_WEBCAM_MSG_WEBCAM_SUB_HPP_
#define INCLUDE_WEBCAM_MSG_WEBCAM_SUB_HPP_

#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "webcam_msg/webcam_msg_lib.hpp"

std::vector<float> byteArrayToFloatArray(const std::vector<uint8_t>& byteArray);

class SubscriberNode : public rclcpp::Node {
 public:
  SubscriberNode();

 private:
  void message_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
};

#endif  // INCLUDE_WEBCAM_MSG_WEBCAM_SUB_HPP_

