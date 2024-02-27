// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_SEAL_X_ROS_SXR_CLIENT_NODE_HPP_
#define INCLUDE_SEAL_X_ROS_SXR_CLIENT_NODE_HPP_

#include <vector>
#include <memory>
#include <cstdint>
#include <functional>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "seal/seal.h"

#include "seal_x_ros/srv/key_exchange.hpp"
#include "seal_x_ros/srv/operation_request.hpp"
#include "seal_x_ros/srv/server_message.hpp"

#include "seal_x_ros/sxr_parms_and_keys.hpp"
#include "seal_x_ros/sxr_encryptor.hpp"
#include "seal_x_ros/sxr_decryptor.hpp"
#include "seal_x_ros/sxr_lib.hpp"

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
   * operation requests. It also sets up the encryption and decryption environment by initializing
   * SEAL parameters, keys, and encryptor/decryptor objects. This setup enables secure communication
   * and processing of encrypted data.
   */
  SXRClientNode();

 private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription;  ///< ROS 2 subscription for receiving encrypted data.
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher;  ///< ROS 2 publisher for sending encrypted data.
  rclcpp::Client<seal_x_ros::srv::KeyExchange>::SharedPtr key_exchange_client;  ///< Client for key exchange service.
  rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedPtr operation_request_client;  ///< Client for operation request service.
  rclcpp::Service<seal_x_ros::srv::ServerMessage>::SharedPtr server_message_service;  ///< Service for receiving server messages.

  /**
   * @brief Establishes connection with the server and sends encryption keys.
   * 
   * This function waits for the server to become available, and then sends the 
   * serialized encryption parameters and keys to the server. On successful 
   * key exchange, it initiates sending the ciphertext. This is crucial for setting up
   * the secure communication channel between the client and the server for encrypted data processing.
   */
  void connectionAndSendKey();

  /**
   * @brief Callback for processing received messages.
   * 
   * This function is called upon receiving encrypted data messages. It can decrypt
   * the received messages and process or log the results as necessary.
   * 
   * @param msg The received message containing encrypted data.
   */
  void messageCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);

  /**
   * @brief Sends an encrypted ciphertext to the server for processing.
   * 
   * Encrypts a message, sends the resulting ciphertext to the server for processing, 
   * and defines a callback for handling the processed data. This function facilitates 
   * encrypted data transmission for server-side processing and handling of the results.
   * 
   * @param message The message to encrypt and send.
   * @param callback The callback function to execute with the decrypted result.
   */
  void sendCiphertext(std::vector<float> message,
                      std::function<void(const std::vector<float>&)> callback);

  void sendBigCiphertext(std::vector<float> message,
                      std::function<void(const std::vector<float>&)> callback);

  /**
   * @brief Handles messages from the server.
   * 
   * Processes server messages, typically containing encrypted computation results, and
   * responds if necessary. This function ensures that the client node can handle and
   * act upon messages received from the server node.
   * 
   * @param request The request received from the server.
   * @param response The response to be sent to the server.
   */
  void handleServerMessage(const std::shared_ptr<seal_x_ros::srv::ServerMessage::Request> request,
                           std::shared_ptr<seal_x_ros::srv::ServerMessage::Response> response);

  SXRDecryptor decryptor;  ///< Custom decryptor for SEAL-X-ROS framework.
  SXREncryptor encryptor;  ///< Custom encryptor for SEAL-X-ROS framework.
  ParmsAndKeysManager mParmsAndKeys;  ///< Manager for SEAL parameters and keys.

  std::vector<uint8_t> mSerializedParms;  ///< Serialized encryption parameters.
  std::vector<uint8_t> mSerializedPk;  ///< Serialized public key.
  std::vector<uint8_t> mSerializedRlk;  ///< Serialized relinearization keys.
  std::vector<uint8_t> mSerializedGalk;  ///< Serialized Galois keys.
  double mScale;  ///< Scale factor for CKKS encryption scheme.

  seal::EncryptionParameters mParms;  ///< SEAL encryption parameters.
  std::shared_ptr<seal::SEALContext> mpContext;  ///< Pointer to SEAL context.

  std::shared_ptr<seal::PublicKey> mpPublicKey;  ///< Pointer to public key.
  std::shared_ptr<seal::SecretKey> mpSecretKey;  ///< Pointer to secret key.
  std::shared_ptr<seal::RelinKeys> mpRelinKeys;  ///< Pointer to relinearization keys.
  std::shared_ptr<seal::GaloisKeys> mpGaloisKeys;  ///< Pointer to Galois keys.

  std::unique_ptr<seal::Decryptor> mpDecryptor;  ///< Unique pointer to SEAL decryptor.
  std::unique_ptr<seal::CKKSEncoder> mpEncoder;  ///< Unique pointer to CKKS encoder.
  std::unique_ptr<seal::Encryptor> mpEncryptor;  ///< Unique pointer to SEAL encryptor.
};

#endif  // INCLUDE_SEAL_X_ROS_SXR_CLIENT_NODE_HPP_

