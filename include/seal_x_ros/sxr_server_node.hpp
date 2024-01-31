#ifndef SXR_SERVER_NODE_HPP
#define SXR_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "seal/seal.h"

#include "seal_x_ros/sxr_encryptor.hpp"
#include "seal_x_ros/sxr_evaluator.hpp"
#include "seal_x_ros/sxr_lib.hpp"

#include "seal_x_ros/srv/key_exchange.hpp"
#include "seal_x_ros/srv/operation_request.hpp"
#include "seal_x_ros/srv/server_message.hpp"

#include <vector>
#include <memory>
#include <cstdint>

/**
 * @class SXRServerNode
 * @brief ROS 2 Node for SEAL server-side operations in encrypted data processing.
 *
 * This class encapsulates the server-side functionalities for processing encrypted data.
 * It provides services for key exchange and handling encrypted operation requests, facilitating
 * secure and private computations on encrypted data.
 */
class SXRServerNode : public rclcpp::Node {
public:
	/**
	 * @brief Constructs a new SXRServerNode object.
	 *
	 * Initializes the ROS 2 server node along with necessary services for key exchange and 
	 * operation requests. It prepares the environment for performing encrypted operations.
	 */
	SXRServerNode();

private:
	rclcpp::Client<seal_x_ros::srv::ServerMessage>::SharedPtr server_message_client_;
	
	void send_message();
	
	rclcpp::Service<seal_x_ros::srv::KeyExchange>::SharedPtr key_exchange_service_;
	rclcpp::Service<seal_x_ros::srv::OperationRequest>::SharedPtr operation_request_service_;
	
	/**
	 * @brief Handles the key exchange request from the client.
	 * 
	 * This function processes the key exchange request by storing the received serialized
	 * encryption parameters and keys. It sets up the necessary environment for the evaluator
	 * and sends a response back to the client confirming the receipt of the public key.
	 * 
	 * @param request The request object containing serialized encryption parameters and keys.
	 * @param response The response object to be sent back to the client.
	 */
	void handle_key_exchange(const std::shared_ptr<seal_x_ros::srv::KeyExchange::Request> request, std::shared_ptr<seal_x_ros::srv::KeyExchange::Response> response);
	
	/**
	 * @brief Handles an operation request on encrypted data from the client.
	 * 
	 * This function takes the encrypted data from the request, performs the specified
	 * operation using the evaluator, and then sends back the result as encrypted data.
	 * If the evaluator is not initialized, it logs an error and sets the response to indicate failure.
	 * 
	 * @param request The request object containing the serialized ciphertext.
	 * @param response The response object containing the result or error message.
	 */
	void handle_operation_request(const std::shared_ptr<seal_x_ros::srv::OperationRequest::Request> request, std::shared_ptr<seal_x_ros::srv::OperationRequest::Response> response);
	
	std::vector<uint8_t> serialized_parms_;
	std::vector<uint8_t> serialized_pk_;
	std::vector<uint8_t> serialized_rlk_;
	std::vector<uint8_t> serialized_galk_;
	double scale_;
	
	std::optional<SXREncryptor> encryptor_;
	std::optional<SXREvaluator> evaluator_;
};

#endif // SXR_SERVER_NODE_HPP

