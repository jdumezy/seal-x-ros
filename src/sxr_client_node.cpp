#include "seal_x_ros/sxr_client_node.hpp"

SXRClientNode::SXRClientNode()
    : rclcpp::Node("sxr_client_node"),
      parmsAndKeys_(),
      encryptor_(parmsAndKeys_.get_serialized_parms(),
                 parmsAndKeys_.get_serialized_pk(),
                 parmsAndKeys_.get_scale()),
      decryptor_(parmsAndKeys_.get_serialized_parms(),
                 parmsAndKeys_.get_secret_key()) {
    key_exchange_client_ = this->create_client<seal_x_ros::srv::KeyExchange>(
        "key_exchange_service");
    operation_request_client_ = this->create_client<seal_x_ros::srv::OperationRequest>(
        "operation_request_service");
    server_message_service_ = this->create_service<seal_x_ros::srv::ServerMessage>(
        "server_message_service",
        std::bind(&SXRClientNode::handle_server_message, this,
            std::placeholders::_1, std::placeholders::_2));

    serialized_parms_ = parmsAndKeys_.get_serialized_parms();
    serialized_pk_ = parmsAndKeys_.get_serialized_pk();
    serialized_rlk_ = parmsAndKeys_.get_serialized_rlk();
    serialized_galk_ = parmsAndKeys_.get_serialized_galk();
    secret_key_ = parmsAndKeys_.get_secret_key();
    scale_ = parmsAndKeys_.get_scale();

    connection_and_send_key();
}

void SXRClientNode::connection_and_send_key() {
    while (!key_exchange_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
    }

    auto request = std::make_shared<seal_x_ros::srv::KeyExchange::Request>();
    request->serialized_parms = serialized_parms_;
    request->serialized_pk = serialized_pk_;
    request->serialized_rlk = serialized_rlk_;
    request->serialized_galk = serialized_galk_;
    request->scale = scale_;

    key_exchange_client_->async_send_request(request,
        [this](rclcpp::Client<seal_x_ros::srv::KeyExchange>::SharedFuture future_response) {
            auto response = future_response.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Key exchange successful");
                send_ciphertext();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Key exchange failed");
            }
    });
}

void SXRClientNode::send_ciphertext() {
    float f = 3.0f;
    std::vector<uint8_t> serializedCt = encryptor_.encrypt_float(f);

    RCLCPP_DEBUG(this->get_logger(), "Sending ciphertext: %f", f);

    auto request = std::make_shared<seal_x_ros::srv::OperationRequest::Request>();
    request->serialized_ct = serializedCt;

    operation_request_client_->async_send_request(request,
        [this](rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedFuture future_response) {
            auto response = future_response.get();
            if (response->success) {
                RCLCPP_DEBUG(this->get_logger(), "Ciphertext received");
                std::vector<uint8_t> serializedCtRes = response->serialized_ct_res;
                float result = decryptor_.decryptFloat(serializedCtRes);
                RCLCPP_INFO(this->get_logger(), "Received response: %f", result);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send ciphertext");
            }
    });
}

void SXRClientNode::handle_server_message(const std::shared_ptr<seal_x_ros::srv::ServerMessage::Request> request,
    std::shared_ptr<seal_x_ros::srv::ServerMessage::Response> response) {

    std::vector<uint8_t> serializedCt = request->serialized_ct;

    RCLCPP_INFO(this->get_logger(), "Received message from server");

    response->success = true;

    float message = decryptor_.decryptFloat(serializedCt);
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

