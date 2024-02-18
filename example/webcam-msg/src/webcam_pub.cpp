// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "webcam_msg/webcam_pub.hpp"

using std::placeholders::_1;

PublisherNode::PublisherNode() : Node("data_pub") {
  publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(
    "sxr_input", 10);
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PublisherNode::publish_message, this));
}

void PublisherNode::publish_message() {
  std::tuple<std::vector<float>, int, int> r = acquireWebcam();
  int width = std::get<1>(r);
  int height = std::get<2>(r);
  std::vector<float> floatArray = std::get<0>(r);

  displayFloatArray(floatArray, width, height, "Input stream");

  RCLCPP_INFO(this->get_logger(), "Sending Float Array %d %d", width, height);

  auto message = std_msgs::msg::ByteMultiArray();
  message.data = floatArrayToByteArray(floatArray);
  publisher_->publish(message);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

