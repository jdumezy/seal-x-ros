#ifndef SEAL_CLIENT_NODE_HPP
#define SEAL_CLIENT_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "seal_ros_nodes/seal_parms_and_keys.hpp"
#include "seal_ros_nodes/seal_encryptor.hpp"
#include "seal_ros_nodes/seal_decryptor.hpp"

#include "seal/seal.h"

#include "std_msgs/msg/string.hpp"
#include "seal_msgs/msg/seal_data.hpp"
#include "seal_msgs/srv/key_exchange.hpp"

#include <vector>
#include <memory>

class SealClientNode : public rclcpp::Node {
public:
	SealClientNode();

private:
	void connection_and_send_key();
	void send_ciphertext();
	void response_callback(const std_msgs::msg::String::SharedPtr msg);

	rclcpp::Client<seal_msgs::srv::KeyExchange>::SharedPtr key_exchange_client_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ciphertext_pub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr response_sub_;
	
	std::vector<uint8_t> parms_;
	std::vector<uint8_t> public_key_;
	seal::SecretKey secret_key_;
	double scale_;
};

#endif // SEAL_CLIENT_NODE_HPP

