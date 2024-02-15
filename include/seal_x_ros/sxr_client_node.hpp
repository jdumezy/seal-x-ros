#ifndef SXR_CLIENT_NODE_HPP_
#define SXR_CLIENT_NODE_HPP_

#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "seal/seal.h"

#include "seal_x_ros/sxr_parms_and_keys.hpp"
#include "seal_x_ros/sxr_encryptor.hpp"
#include "seal_x_ros/sxr_decryptor.hpp"
#include "seal_x_ros/sxr_lib.hpp"

#include "seal_x_ros/srv/key_exchange.hpp"
#include "seal_x_ros/srv/operation_request.hpp"
#include "seal_x_ros/srv/server_message.hpp"
#include "sxr_encryptor.hpp"

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
  SXRDecryptor decryptor;
  SXREncryptor encryptor;

private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher;
  rclcpp::Client<seal_x_ros::srv::KeyExchange>::SharedPtr key_exchange_client;
  rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedPtr operation_request_client;
  rclcpp::Service<seal_x_ros::srv::ServerMessage>::SharedPtr server_message_service;

  /**
   * @brief Establishes connection with the server and sends encryption keys.
   * 
   * This function waits for the server to become available, and then sends the 
   * serialized encryption parameters and keys to the server. On successful 
   * key exchange, it initiates sending the ciphertext.
   */
  void connectionAndSendKey();
  
  void messageCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);

  /**
   * @brief Sends an encrypted ciphertext to the server for processing.
   * 
   * Encrypts a predefined float value, then sends the resulting ciphertext
   * to the server for processing. Upon receiving the processed ciphertext
   * back from the server, it decrypts and logs the result.
   */
  void sendCiphertext(std::vector<float> message, std::function<void(const std::vector<float>&)> callback);

  void handleServerMessage(const std::shared_ptr<seal_x_ros::srv::ServerMessage::Request> request, std::shared_ptr<seal_x_ros::srv::ServerMessage::Response> response);
  
  ParmsAndKeysManager mParmsAndKeys;
  std::vector<uint8_t> mSerializedParms;
  std::vector<uint8_t> mSerializedPk;
  std::vector<uint8_t> mSerializedRlk;
  std::vector<uint8_t> mSerializedGalk;
  double mScale;

  seal::EncryptionParameters mParms;
  std::shared_ptr<seal::SEALContext> mpContext;

  std::shared_ptr<seal::PublicKey> mpPublicKey;
  std::shared_ptr<seal::SecretKey> mpSecretKey;
  std::shared_ptr<seal::RelinKeys> mpRelinKeys;
  std::shared_ptr<seal::GaloisKeys> mpGaloisKeys;

  std::unique_ptr<seal::Decryptor> mpDecryptor;
  std::unique_ptr<seal::CKKSEncoder> mpEncoder;
  std::unique_ptr<seal::Encryptor> mpEncryptor;

  SXRDecryptor mDecryptor;
};

#endif // SXR_CLIENT_NODE_HPP_

