#ifndef SEAL_CLIENT_NODE_HPP
#define SEAL_CLIENT_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "seal_ros_nodes/seal_parms_and_keys.hpp"
#include "seal_ros_nodes/seal_encryptor.hpp"
#include "seal_ros_nodes/seal_decryptor.hpp"
#include "seal_ros_nodes/sxr_lib.hpp"

#include "seal/seal.h"

#include "seal_msgs/srv/key_exchange.hpp"
#include "seal_msgs/srv/operation_request.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class SealClientNode : public rclcpp::Node {
public:
	SealClientNode();

private:
	void connection_and_send_key();
	void send_ciphertext();

	rclcpp::Client<seal_msgs::srv::KeyExchange>::SharedPtr key_exchange_client_;
	rclcpp::Client<seal_msgs::srv::OperationRequest>::SharedPtr operation_request_client_;
	
	ParmsAndKeysManager parmsAndKeys_;
	std::vector<uint8_t> serialized_parms_;
	std::vector<uint8_t> serialized_pk_;
	std::vector<uint8_t> serialized_rlk_;
	std::vector<uint8_t> serialized_galk_;
	seal::SecretKey secret_key_;
	double scale_;
	
	EncryptorManager encryptor_;
	DecryptorManager decryptor_;
};

#endif // SEAL_CLIENT_NODE_HPP

