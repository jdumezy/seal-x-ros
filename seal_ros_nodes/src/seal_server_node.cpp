#include "seal_ros_nodes/seal_server_node.hpp"

SealServerNode::SealServerNode() : Node("seal_server_node")
{
    key_exchange_service_ = this->create_service<seal_msgs::srv::KeyExchange>(
        "key_exchange_service",
        std::bind(&SealServerNode::handle_key_exchange, this, std::placeholders::_1, std::placeholders::_2));

    ciphertext_sub_ = this->create_subscription<std_msgs::msg::String>(
        "seal_ciphertext_topic",
        10,
        std::bind(&SealServerNode::ciphertext_callback, this, std::placeholders::_1));
}

void SealServerNode::handle_key_exchange(const std::shared_ptr<seal_msgs::srv::KeyExchange::Request> request,
                                         std::shared_ptr<seal_msgs::srv::KeyExchange::Response> response)
{
    public_key_ = request->public_key;
    RCLCPP_INFO(this->get_logger(), "Received public key: %s", public_key_.c_str());

    
    response->success = true;
    response->message = "Public key received successfully";
}

void SealServerNode::ciphertext_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received ciphertext: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SealServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

