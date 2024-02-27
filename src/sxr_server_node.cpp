// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_server_node.hpp"

SXRServerNode::SXRServerNode()
  : Node("sxr_server_node"), encryptor(), evaluator() {
  server_message_client = this->create_client<seal_x_ros::srv::ServerMessage>(
    "server_message_service");

  key_exchange_service = this->create_service<seal_x_ros::srv::KeyExchange>(
    "key_exchange_service",
    std::bind(&SXRServerNode::handleKeyExchange, this,
              std::placeholders::_1, std::placeholders::_2));

  operation_request_service = this->create_service<seal_x_ros::srv::OperationRequest>(
    "operation_request_service",
    std::bind(&SXRServerNode::handleOperationRequest, this,
              std::placeholders::_1, std::placeholders::_2));
}

void SXRServerNode::sendMessage() {
  auto request = std::make_shared<seal_x_ros::srv::ServerMessage::Request>();

  float f = 3.1415f;  // TODO(jdumezy) add entry message instead of static float
  std::vector<uint8_t> serializedCt = encryptor.encryptFloat(f);

  if (encryptor.isInit()) {
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
  mScale = request->scale;

  mParms = deserializeToParms(request->serialized_parms);
  mpContext = std::make_shared<seal::SEALContext>(mParms);

  mpPublicKey = std::make_shared<seal::PublicKey>(
    deserializeToPk(request->serialized_pk, mpContext.get()));
  mpRelinKeys = std::make_shared<seal::RelinKeys>(
    deserializeToRlk(request->serialized_rlk, mpContext.get()));
  mpGaloisKeys = std::make_shared<seal::GaloisKeys>(
    deserializeToGalk(request->serialized_galk, mpContext.get()));

  mpEncoder = std::make_unique<seal::CKKSEncoder>(*mpContext.get());
  mpEncryptor = std::make_unique<seal::Encryptor>(*mpContext.get(),
                                                  *mpPublicKey.get());
  mpEvaluator = std::make_unique<seal::Evaluator>(*mpContext.get());

  encryptor.init(mpEncoder.get(), mpEncryptor.get(), mScale);
  evaluator.init(mpEncoder.get(), mpEncryptor.get(), mpEvaluator.get(),
                 mpRelinKeys.get(), mpGaloisKeys.get(), mScale);

  RCLCPP_INFO(this->get_logger(), "Received SEAL parameters");

  response->success = true;
}

void SXRServerNode::handleOperationRequest(const std::shared_ptr<seal_x_ros::srv::OperationRequest::Request> request,
                                           std::shared_ptr<seal_x_ros::srv::OperationRequest::Response> response) {
  RCLCPP_DEBUG(this->get_logger(), "Received ciphertext");

  seal::Ciphertext requestCiphertext = deserializeToCt(request->serialized_ct,
                                                       mpContext.get());

  if (evaluator.isInit()) {
    SXRCiphertext message(requestCiphertext);

    SXRCiphertext result = gray_scale(message, &evaluator);

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

