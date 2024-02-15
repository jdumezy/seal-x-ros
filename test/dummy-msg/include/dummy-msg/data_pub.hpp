#ifndef DATA_PUB_HPP_
#define DATA_PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include <vector>
#include <cstring>

std::vector<uint8_t> floatArrayToByteArray(const std::vector<float>& floatArray);

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode();

private:
  void publish_message();
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DATA_PUB_HPP_

