// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_server_node.hpp"

SXRServerNode::SXRServerNode()
  : Node("sxr_server_node") {
  key_exchange_service = this->create_service<seal_x_ros::srv::KeyExchange>(
    "key_exchange_service",
    std::bind(&SXRServerNode::handleKeyExchange, this,
      std::placeholders::_1, std::placeholders::_2));

  operation_request_service = this->create_service<seal_x_ros::srv::OperationRequest>(
    "operation_request_service",
    std::bind(&SXRServerNode::handleOperationRequest, this,
      std::placeholders::_1, std::placeholders::_2));
  server_message_client = this->create_client<seal_x_ros::srv::ServerMessage>(
    "server_message_service");
}

void SXRServerNode::sendMessage() {
  auto request = std::make_shared<seal_x_ros::srv::ServerMessage::Request>();

  float f = 3.1415f;  //TODO(jdumezy) add entry message instead of float
  std::vector<uint8_t> serializedCt = mEncryptor->encryptFloat(f);

  if (mEncryptor) {
    request->serialized_ct = serializedCt;

    server_message_client->async_send_request(request,
      [this](rclcpp::Client<seal_x_ros::srv::ServerMessage>::SharedFuture futureResponse) {
        auto response = futureResponse.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "Message sent successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Message sending failed");
        }
    });
  } else {
    RCLCPP_ERROR(this->get_logger(), "Encryptor is not initialized");
  }
}

void SXRServerNode::handleKeyExchange(const std::shared_ptr<seal_x_ros::srv::KeyExchange::Request> request,
  std::shared_ptr<seal_x_ros::srv::KeyExchange::Response> response) {

  mSerializedParms = request->serialized_parms;
  mSerializedPk = request->serialized_pk;
  mSerializedRlk = request->serialized_rlk;
  mSerializedGalk = request->serialized_galk;
  mScale = request->scale;

  RCLCPP_INFO(this->get_logger(), "Received public key");

  response->success = true;

  // Initialise SXR objects
  mEncryptor.emplace(mSerializedParms, mSerializedPk, mScale);
  mEvaluator.emplace(mSerializedParms, mSerializedPk, mSerializedRlk, mSerializedGalk, mScale);

  // Initialise SEAL shared objects
  mParms = deserializeToParms(mSerializedParms);

  seal::SEALContext context(mParms);
  std::shared_ptr<seal::SEALContext> pContext = std::make_shared<seal::SEALContext>(mParms);

  mPublicKey = deserializeToPk(mSerializedPk, pContext);
  mRelinKeys = deserializeToRlk(mSerializedRlk, pContext);
  mGaloisKeys = deserializeToGalk(mSerializedGalk, pContext);
}

void SXRServerNode::handleOperationRequest(const std::shared_ptr<seal_x_ros::srv::OperationRequest::Request> request,
  std::shared_ptr<seal_x_ros::srv::OperationRequest::Response> response) {
  RCLCPP_DEBUG(this->get_logger(), "Received ciphertext");

  seal::SEALContext context(mParms);  // TODO(jdumezy) remove + change classes
  std::shared_ptr<seal::SEALContext> pContext = std::make_shared<seal::SEALContext>(mParms);
  seal::Ciphertext requestCiphertext = deserializeToCt(request->serialized_ct, pContext);

  if (mEvaluator) {
    SXRCiphertext message(requestCiphertext);

    SXRCiphertext result = mEvaluator->add(message, message);  //TODO(jdumezy) add default constructor for later init

    RCLCPP_INFO(this->get_logger(), "Sending processed ciphertext");
    response->success = true;
    response->serialized_ct_res = serializeSealObject(result.getCiphertext());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Evaluator is not initialized");
    response->success = false;
  }
}

/**
 * @brief The main function for the SXRServerNode.
 * 
 * Initializes the ROS 2 node and spins it to start processing callbacks for
 * key exchange and operation requests.
 * 
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return Integer representing the status at the end of execution.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SXRServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

