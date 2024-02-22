// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "webcam_msg/webcam_sub.hpp"

using std::placeholders::_1;

SubscriberNode::SubscriberNode() : Node("data_sub") {
  subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
    "sxr_output", 10, std::bind(&SubscriberNode::message_callback, this, _1));
}

void SubscriberNode::message_callback(
  const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
  std::vector<float> floatArray = byteArrayToFloatArray(msg->data);

  int width = 640;
  int height = 480;

  displayFloatArray(floatArray, width, height, "Output stream");

  RCLCPP_INFO(this->get_logger(), "Received Float Array");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

