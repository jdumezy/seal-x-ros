#include "dummy-msg/data_sub.hpp"

using std::placeholders::_1;

std::vector<float> byteArrayToFloatArray(const std::vector<uint8_t>& byteArray) {
  std::vector<float> floatArray;
  size_t len = byteArray.size() / sizeof(float);
  floatArray.resize(len);

  for (size_t i = 0; i < len; ++i) {
    std::memcpy(&floatArray[i], &byteArray[i * sizeof(float)], sizeof(float));
  }

  return floatArray;
}

SubscriberNode::SubscriberNode() : Node("data_sub") {
  subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
    "sxr_output", 10, std::bind(&SubscriberNode::message_callback, this, _1));
}

void SubscriberNode::message_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
  std::vector<float> floatArray = byteArrayToFloatArray(msg->data);
  
  RCLCPP_INFO(this->get_logger(), "Received Float Array: [");
  for (int i = 0; i < 4; i++) {
    RCLCPP_INFO(this->get_logger(), " %f ", floatArray[i]);
  }
  RCLCPP_INFO(this->get_logger(), "]\n");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

