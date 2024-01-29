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

/**
 * @class SXRClientNode
 * @brief ROS 2 Node for SEAL client-side operations in encrypted data processing.
 *
 * This class represents a ROS 2 Node handling the client-side functionalities in an encrypted
 * data processing system. It manages key exchanges, sends, and receives encrypted data
 * with a corresponding server node for processing requests.
 */
class SXRClientNode : public rclcpp::Node {
public:
	/**
	 * @brief Constructs a new SXRClientNode object.
	 *
	 * Initializes the ROS 2 client node along with necessary service clients for key exchange and
	 * operation requests. It also sets up the encryption and decryption environment.
	 */
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

