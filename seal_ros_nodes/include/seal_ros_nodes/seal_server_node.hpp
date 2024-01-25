#ifndef SEAL_SERVER_NODE_HPP
#define SEAL_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "seal/seal.h"

#include "std_msgs/msg/string.hpp"
#include "seal_msgs/msg/seal_data.hpp"
#include "seal_msgs/srv/key_exchange.hpp"

#include <string>

class SealServerNode : public rclcpp::Node {
public:
	SealServerNode();

private:
	void handle_key_exchange(const std::shared_ptr<seal_msgs::srv::KeyExchange::Request> request, std::shared_ptr<seal_msgs::srv::KeyExchange::Response> response);
	void ciphertext_callback(const std_msgs::msg::String::SharedPtr msg);

	rclcpp::Service<seal_msgs::srv::KeyExchange>::SharedPtr key_exchange_service_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ciphertext_sub_;
	std::string public_key_;
};

#endif // SEAL_SERVER_NODE_HPP

