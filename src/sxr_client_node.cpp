// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_client_node.hpp"

SXRClientNode::SXRClientNode()
  : rclcpp::Node("sxr_client_node"),
    mParmsAndKeys(),
    mEncryptor(mParmsAndKeys.getSerializedParms(),
         mParmsAndKeys.getSerializedPk(),
         mParmsAndKeys.getScale()),
    mDecryptor(mParmsAndKeys.getSerializedParms(),
         mParmsAndKeys.getSecretKey()) {
  key_exchange_client = this->create_client<seal_x_ros::srv::KeyExchange>(
    "key_exchange_service");
  operation_request_client = this->create_client<seal_x_ros::srv::OperationRequest>(
    "operation_request_service");
  server_message_service = this->create_service<seal_x_ros::srv::ServerMessage>(
    "server_message_service",
    std::bind(&SXRClientNode::handleServerMessage, this,
      std::placeholders::_1, std::placeholders::_2));
  subscription = this->create_subscription<std_msgs::msg::ByteMultiArray>(
    "sxr_input", 10, std::bind(&SXRClientNode::messageCallback, this, std::placeholders::_1));
  publisher = this->create_publisher<std_msgs::msg::ByteMultiArray>("sxr_output", 10);


  mSerializedParms = mParmsAndKeys.getSerializedParms();
  mSerializedPk = mParmsAndKeys.getSerializedPk();
  mSerializedRlk = mParmsAndKeys.getSerializedRlk();
  mSerializedGalk = mParmsAndKeys.getSerializedGalk();
  mSecretKey = mParmsAndKeys.getSecretKey();
  mScale = mParmsAndKeys.getScale();

  connectionAndSendKey();
}

void SXRClientNode::connectionAndSendKey() {
  while (!key_exchange_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
  }

  auto request = std::make_shared<seal_x_ros::srv::KeyExchange::Request>();
  request->serialized_parms = mSerializedParms;
  request->serialized_pk = mSerializedPk;
  request->serialized_rlk = mSerializedRlk;
  request->serialized_galk = mSerializedGalk;
  request->scale = mScale;

  key_exchange_client->async_send_request(request,
    [this](rclcpp::Client<seal_x_ros::srv::KeyExchange>::SharedFuture futureResponse) {
      auto response = futureResponse.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Key exchange successful");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Key exchange failed");
      }
  });
}

void SXRClientNode::messageCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received data from outside node");
  std::vector<float> message = byteArrayToFloatArray(msg->data);
  size_t originalSize = message.size();

  sendCiphertext(message, [this, originalSize](const std::vector<float>& result) {
    std::vector<float> trimmedResult = result;
    if (trimmedResult.size() > originalSize) {
      trimmedResult.resize(originalSize);
    }
    auto result_msg = std_msgs::msg::ByteMultiArray();
    result_msg.data = floatArrayToByteArray(trimmedResult);
    publisher->publish(result_msg);
    RCLCPP_INFO(this->get_logger(), "Received processed ciphertext");
  });
}

void SXRClientNode::sendCiphertext(std::vector<float> message, std::function<void(const std::vector<float>&)> callback) {
  std::vector<uint8_t> serializedCt = mEncryptor.encryptFloatArray(message);

  RCLCPP_DEBUG(this->get_logger(), "Sending ciphertext");

  auto request = std::make_shared<seal_x_ros::srv::OperationRequest::Request>();
  request->serialized_ct = serializedCt;

  operation_request_client->async_send_request(request,
    [this, callback](rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedFuture futureResponse) {
      auto response = futureResponse.get();
      if (response->success) {
        RCLCPP_DEBUG(this->get_logger(), "Ciphertext received");
        std::vector<uint8_t> serializedCtRes = response->serialized_ct_res;
        std::vector<float> result = { mDecryptor.decryptFloatArray(serializedCtRes) };
        callback(result);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send ciphertext");
        callback(std::vector<float>());
      }
  });
}

void SXRClientNode::handleServerMessage(const std::shared_ptr<seal_x_ros::srv::ServerMessage::Request> request,
  std::shared_ptr<seal_x_ros::srv::ServerMessage::Response> response) {
  std::vector<uint8_t> serializedCt = request->serialized_ct;

  RCLCPP_INFO(this->get_logger(), "Received message from server");

  response->success = true;

  float message = mDecryptor.decryptFloat(serializedCt);
  message++;
}

/**
 * @brief The main function for the SXRClientNode.
 * 
 * Initializes the ROS 2 node and spins it to start processing service requests
 * for key exchange and sending encrypted data to the server.
 * 
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return Integer representing the status at the end of execution.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SXRClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

