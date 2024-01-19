#ifndef SEAL_CLIENT_NODE_HPP
#define SEAL_CLIENT_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "seal_msgs/msg/seal_data.hpp"

class SealClientNode : public rclcpp::Node {
public:
    SealClientNode();

private:
    void test_seal_function();
    void decrypted_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Publisher<seal_msgs::msg::SealData>::SharedPtr encrypted_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decrypted_subscriber;
};

#endif // SEAL_CLIENT_NODE_HPP

