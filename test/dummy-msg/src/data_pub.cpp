#include "dummy-msg/data_pub.hpp"

using std::placeholders::_1;

std::vector<uint8_t> floatArrayToByteArray(const std::vector<float>& floatArray) {
  std::vector<uint8_t> byteArray;
  byteArray.reserve(floatArray.size() * sizeof(float));

  for (const float& value : floatArray) {
    uint8_t temp[sizeof(float)];
    std::memcpy(temp, &value, sizeof(float));

    byteArray.insert(byteArray.end(), temp, temp + sizeof(float));
  }

  return byteArray;
}

PublisherNode::PublisherNode() : Node("data_pub") {
  publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("sxr_input", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PublisherNode::publish_message, this));
}

void PublisherNode::publish_message() {
  std::vector<float> floatArray = {3.1415, 42.0, 2.0, 1.0};

  RCLCPP_INFO(this->get_logger(), "Sending Float Array: [");
  for (const float& value : floatArray) {
    RCLCPP_INFO(this->get_logger(), " %f ", value);
  }
  RCLCPP_INFO(this->get_logger(), "]\n");

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
