#ifndef SXR_CLIENT_NODE_HPP
#define SXR_CLIENT_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "seal/seal.h"

#include "seal_x_ros/sxr_parms_and_keys.hpp"
#include "seal_x_ros/sxr_encryptor.hpp"
#include "seal_x_ros/sxr_decryptor.hpp"
#include "seal_x_ros/sxr_lib.hpp"

#include "seal_x_ros/srv/key_exchange.hpp"
#include "seal_x_ros/srv/operation_request.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class SXRClientNode : public rclcpp::Node {
public:
	SXRClientNode();

private:
	void connection_and_send_key();
	void send_ciphertext();

	rclcpp::Client<seal_x_ros::srv::KeyExchange>::SharedPtr key_exchange_client_;
	rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedPtr operation_request_client_;
	
	ParmsAndKeysManager parmsAndKeys_;
	std::vector<uint8_t> serialized_parms_;
	std::vector<uint8_t> serialized_pk_;
	std::vector<uint8_t> serialized_rlk_;
	std::vector<uint8_t> serialized_galk_;
	seal::SecretKey secret_key_;
	double scale_;
	
	SXREncryptor encryptor_;
	SXRDecryptor decryptor_;
};

#endif // SXR_CLIENT_NODE_HPP

