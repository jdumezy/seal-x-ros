// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_SEAL_X_ROS_SXR_SERVER_NODE_HPP_
#define INCLUDE_SEAL_X_ROS_SXR_SERVER_NODE_HPP_

#include <vector>
#include <memory>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"

#include "seal/seal.h"

#include "seal_x_ros/srv/server_message.hpp"
#include "seal_x_ros/srv/key_exchange.hpp"
#include "seal_x_ros/srv/operation_request.hpp"

#include "seal_x_ros/sxr_ciphertext.hpp"
#include "seal_x_ros/sxr_encryptor.hpp"
#include "seal_x_ros/sxr_evaluator.hpp"
#include "seal_x_ros/sxr_lib.hpp"

/**
 * @class SXRServerNode
 * @brief ROS 2 Node for SEAL server-side operations in encrypted data processing.
 *
 * Encapsulates server-side functionalities for encrypted data processing within a ROS 2 framework.
 * It facilitates secure and private computations on encrypted data by offering services for key exchange
 * and encrypted operation requests. The node leverages Microsoft SEAL for cryptographic operations, ensuring
 * data privacy and security throughout the processing lifecycle.
 */
class SXRServerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs a new SXRServerNode object.
   *
   * Sets up the ROS 2 server node, including initialization of services for key exchange and
   * encrypted operation requests. Prepares the SEAL environment for secure data processing,
   * enabling the server to perform operations on encrypted data received from clients.
   */
  SXRServerNode();

  SXREncryptor encryptor;  ///< Encryptor for performing encryption tasks.
  SXREvaluator evaluator;  ///< Evaluator for performing operations on encrypted data.

 private:
  rclcpp::Client<seal_x_ros::srv::ServerMessage>::SharedPtr server_message_client;  ///< Client for sending messages to the server.
  rclcpp::Service<seal_x_ros::srv::KeyExchange>::SharedPtr key_exchange_service;  ///< Service for handling key exchange requests.
  rclcpp::Service<seal_x_ros::srv::OperationRequest>::SharedPtr operation_request_service;  ///< Service for handling encrypted operation requests.

  void sendMessage();

  /**
   * @brief Handles the key exchange request from the client.
   * 
   * Upon receiving a key exchange request, this method stores the client's encryption parameters
   * and keys, setting up the server's SEAL environment for encrypted operations. It confirms
   * the key exchange by sending an acknowledgment to the client, enabling subsequent encrypted
   * communication and processing.
   * 
   * @param request The incoming request containing the client's encryption parameters and keys.
   * @param response The response to be sent back, confirming the successful exchange.
   */
  void handleKeyExchange(const std::shared_ptr<seal_x_ros::srv::KeyExchange::Request> request,
                         std::shared_ptr<seal_x_ros::srv::KeyExchange::Response> response);

  /**
   * @brief Handles encrypted operation requests from clients.
   * 
   * Processes requests to perform specific operations on encrypted data. Utilizes the evaluator
   * to compute the requested operation on the ciphertexts provided by the client, then returns
   * the encrypted result. If the evaluator is not properly initialized, logs an error and
   * informs the client of the failure.
   * 
   * @param request The request containing the operation type and encrypted data.
   * @param response The response containing the encrypted result or an error message.
   */
  void handleOperationRequest(const std::shared_ptr<seal_x_ros::srv::OperationRequest::Request> request,
                              std::shared_ptr<seal_x_ros::srv::OperationRequest::Response> response);

  double mScale;  ///< Scale factor used in encoding for encryption.

  seal::EncryptionParameters mParms;  ///< Encryption parameters for the SEAL context.
  std::shared_ptr<seal::SEALContext> mpContext;  ///< Shared pointer to the SEAL context.

  std::shared_ptr<seal::PublicKey> mpPublicKey;  ///< Shared pointer to the public key.
  std::shared_ptr<seal::RelinKeys> mpRelinKeys;  ///< Shared pointer to the relinearization keys.
  std::shared_ptr<seal::GaloisKeys> mpGaloisKeys;  ///< Shared pointer to the Galois keys.

  std::unique_ptr<seal::CKKSEncoder> mpEncoder;  ///< Unique pointer to the CKKS encoder.
  std::unique_ptr<seal::Encryptor> mpEncryptor;  ///< Unique pointer to the encryptor.
  std::unique_ptr<seal::Evaluator> mpEvaluator;  ///< Unique pointer to the evaluator for encrypted data operations.
};

#endif  // INCLUDE_SEAL_X_ROS_SXR_SERVER_NODE_HPP_

