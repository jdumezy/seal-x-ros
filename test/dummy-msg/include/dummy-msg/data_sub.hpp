#ifndef DATA_SUB_HPP_
#define DATA_SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include <vector>
#include <cstring>

std::vector<float> byteArrayToFloatArray(const std::vector<uint8_t>& byteArray);

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode();

private:
  void message_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
};

#endif  // DATA_SUB_HPP_

