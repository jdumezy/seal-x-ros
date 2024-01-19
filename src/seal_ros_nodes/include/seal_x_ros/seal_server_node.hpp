#ifndef SEAL_SERVER_NODE_HPP
#define SEAL_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "seal_msgs/msg/seal_data.hpp"

class SealServerNode : public rclcpp::Node {
public:
    SealServerNode();

private:
    void encrypted_callback(const seal_msgs::msg::SealData::SharedPtr msg);

    rclcpp::Subscription<seal_msgs::msg::SealData>::SharedPtr encrypted_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decrypted_publisher;
};

#endif // SEAL_SERVER_NODE_HPP

