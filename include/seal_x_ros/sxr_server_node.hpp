#ifndef SXR_SERVER_NODE_HPP_
#define SXR_SERVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "seal/seal.h"

#include "seal_x_ros/sxr_ciphertext.hpp"
#include "seal_x_ros/sxr_encryptor.hpp"
#include "seal_x_ros/sxr_evaluator.hpp"
#include "seal_x_ros/sxr_lib.hpp"

#include "seal_x_ros/srv/server_message.hpp"
#include "seal_x_ros/srv/key_exchange.hpp"
#include "seal_x_ros/srv/operation_request.hpp"

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

  SXREncryptor encryptor;
  SXREvaluator evaluator;

private:
  rclcpp::Client<seal_x_ros::srv::ServerMessage>::SharedPtr server_message_client;
  rclcpp::Service<seal_x_ros::srv::KeyExchange>::SharedPtr key_exchange_service;
  rclcpp::Service<seal_x_ros::srv::OperationRequest>::SharedPtr operation_request_service;


  void sendMessage();
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
  void handleKeyExchange(const std::shared_ptr<seal_x_ros::srv::KeyExchange::Request> request, std::shared_ptr<seal_x_ros::srv::KeyExchange::Response> response);
  
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
  void handleOperationRequest(const std::shared_ptr<seal_x_ros::srv::OperationRequest::Request> request, std::shared_ptr<seal_x_ros::srv::OperationRequest::Response> response);

  double mScale;
  
  seal::EncryptionParameters mParms;
  std::shared_ptr<seal::SEALContext> mpContext;

  std::shared_ptr<seal::PublicKey> mpPublicKey;
  std::shared_ptr<seal::RelinKeys> mpRelinKeys;
  std::shared_ptr<seal::GaloisKeys> mpGaloisKeys;

  std::unique_ptr<seal::CKKSEncoder> mpEncoder;
  std::unique_ptr<seal::Encryptor> mpEncryptor;
  std::unique_ptr<seal::Evaluator> mpEvaluator;
};

#endif // SXR_SERVER_NODE_HPP_

