#ifndef SXR_CLIENT_NODE_HPP_
#define SXR_CLIENT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "seal/seal.h"

#include "seal_x_ros/sxr_parms_and_keys.hpp"
#include "seal_x_ros/sxr_encryptor.hpp"
#include "seal_x_ros/sxr_decryptor.hpp"
#include "seal_x_ros/sxr_lib.hpp"

#include "seal_x_ros/srv/key_exchange.hpp"
#include "seal_x_ros/srv/operation_request.hpp"
#include "seal_x_ros/srv/server_message.hpp"

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
    rclcpp::Client<seal_x_ros::srv::KeyExchange>::SharedPtr key_exchange_client;
    rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedPtr operation_request_client;
    
    /**
     * @brief Establishes connection with the server and sends encryption keys.
     * 
     * This function waits for the server to become available, and then sends the 
     * serialized encryption parameters and keys to the server. On successful 
     * key exchange, it initiates sending the ciphertext.
     */
    void connectionAndSendKey();
    
    /**
     * @brief Sends an encrypted ciphertext to the server for processing.
     * 
     * Encrypts a predefined float value, then sends the resulting ciphertext
     * to the server for processing. Upon receiving the processed ciphertext
     * back from the server, it decrypts and logs the result.
     */
    void sendCiphertext();
    
    rclcpp::Service<seal_x_ros::srv::ServerMessage>::SharedPtr server_message_service;
    
    void handleServerMessage(const std::shared_ptr<seal_x_ros::srv::ServerMessage::Request> request, std::shared_ptr<seal_x_ros::srv::ServerMessage::Response> response);
    
    ParmsAndKeysManager mParmsAndKeys;
    std::vector<uint8_t> mSerializedParms;
    std::vector<uint8_t> mSerializedPk;
    std::vector<uint8_t> mSerializedRlk;
    std::vector<uint8_t> mSerializedGalk;
    seal::SecretKey mSecretKey;
    double mScale;
    
    SXREncryptor mEncryptor;
    SXRDecryptor mDecryptor;
};

#endif // SXR_CLIENT_NODE_HPP_

