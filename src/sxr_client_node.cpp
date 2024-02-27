// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_client_node.hpp"

SXRClientNode::SXRClientNode()
  : rclcpp::Node("sxr_client_node"), decryptor(), encryptor(), mParmsAndKeys() {
  subscription = this->create_subscription<std_msgs::msg::ByteMultiArray>(
    "sxr_input", 10, std::bind(&SXRClientNode::messageCallback,
                               this, std::placeholders::_1));

  publisher = this->create_publisher<std_msgs::msg::ByteMultiArray>(
    "sxr_output", 10);

  key_exchange_client = this->create_client<seal_x_ros::srv::KeyExchange>(
    "key_exchange_service");

  operation_request_client = this->create_client<seal_x_ros::srv::OperationRequest>(
    "operation_request_service");

  server_message_service = this->create_service<seal_x_ros::srv::ServerMessage>(
    "server_message_service",
    std::bind(&SXRClientNode::handleServerMessage, this,
              std::placeholders::_1, std::placeholders::_2));

  mSerializedParms = mParmsAndKeys.getSerializedParms();
  mSerializedPk = mParmsAndKeys.getSerializedPk();
  mSerializedRlk = mParmsAndKeys.getSerializedRlk();
  mSerializedGalk = mParmsAndKeys.getSerializedGalk();

  mParms = deserializeToParms(mSerializedParms);
  mpContext = std::make_shared<seal::SEALContext>(mParms);

  mpPublicKey = std::make_shared<seal::PublicKey>(
    deserializeToPk(mSerializedPk, mpContext.get()));
  mpSecretKey = std::make_shared<seal::SecretKey>(mParmsAndKeys.getSecretKey());
  mpRelinKeys = std::make_shared<seal::RelinKeys>(
    deserializeToRlk(mSerializedRlk, mpContext.get()));
  mpGaloisKeys = std::make_shared<seal::GaloisKeys>(
    deserializeToGalk(mSerializedGalk, mpContext.get()));

  mpDecryptor = std::make_unique<seal::Decryptor>(*mpContext.get(),
                                                  *mpSecretKey.get());
  mpEncoder = std::make_unique<seal::CKKSEncoder>(*mpContext.get());
  mpEncryptor = std::make_unique<seal::Encryptor>(*mpContext.get(),
                                                  *mpPublicKey.get());
  mScale = mParmsAndKeys.getScale();

  decryptor.init(mpContext.get(), mpDecryptor.get(), mpEncoder.get());
  encryptor.init(mpEncoder.get(), mpEncryptor.get(), mScale);

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

  if (originalSize < mParmsAndKeys.getMaxLen()) {
    sendCiphertext(message, [this, originalSize]
                   (const std::vector<float>& result) {
      std::vector<float> trimmedResult = result;
      if (trimmedResult.size() > originalSize) {
        trimmedResult.resize(originalSize);
      }
      auto result_msg = std_msgs::msg::ByteMultiArray();
      result_msg.data = floatArrayToByteArray(trimmedResult);
      publisher->publish(result_msg);
      RCLCPP_INFO(this->get_logger(), "Received processed ciphertext");
    });
  } else {
    sendBigCiphertext(message, [this, originalSize]  //TODO(jdumezy) clean for DRY
                   (const std::vector<float>& result) {
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
}

void SXRClientNode::sendCiphertext(std::vector<float> message,
                                   std::function<void(const std::vector<float>&)> callback) {
  std::vector<uint8_t> serializedCt = encryptor.encryptFloatArray(message);

  RCLCPP_DEBUG(this->get_logger(), "Sending ciphertext");

  auto request = std::make_shared<seal_x_ros::srv::OperationRequest::Request>();
  request->serialized_ct = serializedCt;

  operation_request_client->async_send_request(request,
    [this, callback](rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedFuture futureResponse) {
      auto response = futureResponse.get();
      if (response->success) {
        RCLCPP_DEBUG(this->get_logger(), "Ciphertext received");
        std::vector<uint8_t> serializedCtRes = response->serialized_ct_res;
        std::vector<float> result = {
          decryptor.decryptFloatArray(serializedCtRes) };
        callback(result);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send ciphertext");
        callback(std::vector<float>());
      }
  });
}

void SXRClientNode::sendBigCiphertext(std::vector<float> message,
                                      std::function<void(const std::vector<float>&)> callback) {

  auto chunks = splitMessage(message, mParmsAndKeys.getMaxLen());
  
  auto encryptedChunks = std::make_shared<std::vector<std::vector<uint8_t>>>();
  encryptedChunks->resize(chunks.size());
  RCLCPP_INFO(this->get_logger(), "Number of chunks: %zu", chunks.size());

  auto results = std::make_shared<std::vector<std::vector<float>>>();
  results->resize(chunks.size());

  auto remaining = std::make_shared<std::atomic<size_t>>(chunks.size());

  for (size_t i = 0; i < chunks.size(); ++i) {
    (*encryptedChunks)[i] = encryptor.encryptFloatArray(chunks[i]);

    auto request = std::make_shared<seal_x_ros::srv::OperationRequest::Request>();
    request->serialized_ct = (*encryptedChunks)[i];

    RCLCPP_DEBUG(this->get_logger(), "Sending ciphertext chunk %zu", i);

    operation_request_client->async_send_request(request,
      [this, i, results, remaining, callback](rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedFuture futureResponse) {
        auto response = futureResponse.get();
        if (response->success) {
          RCLCPP_DEBUG(this->get_logger(), "Ciphertext chunk %zu received", i);
          (*results)[i] = decryptor.decryptFloatArray(response->serialized_ct_res);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to send ciphertext chunk %zu", i);
          (*results)[i] = std::vector<float>();
        }

        size_t remainingCount = remaining->fetch_sub(1) - 1;
        if (remainingCount == 0) {
          auto finalResult = glueMessage(*results);
          RCLCPP_INFO(this->get_logger(), "Chunks reassembled");
          callback(finalResult);
        }
    });
  }
}

void SXRClientNode::handleServerMessage(const std::shared_ptr<seal_x_ros::srv::ServerMessage::Request> request,
                                        std::shared_ptr<seal_x_ros::srv::ServerMessage::Response> response) {
  std::vector<uint8_t> serializedCt = request->serialized_ct;

  RCLCPP_INFO(this->get_logger(), "Received message from server");

  response->success = true;

  float message = decryptor.decryptFloat(serializedCt);
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

