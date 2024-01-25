#include "seal_ros_nodes/seal_client_node.hpp"

SealClientNode::SealClientNode()
	: rclcpp::Node("seal_client_node") {
	
	key_exchange_client_ = this->create_client<seal_msgs::srv::KeyExchange>("key_exchange_service");
	
	ciphertext_pub_ = this->create_publisher<std_msgs::msg::String>("seal_ciphertext_topic", 10);
	
	connection_and_send_key();
}

void SealClientNode::connection_and_send_key() {
	while (!key_exchange_client_->wait_for_service(std::chrono::seconds(1))) {
		RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
	}
	
	auto request = std::make_shared<seal_msgs::srv::KeyExchange::Request>();
	request->public_key = "YourPublicKeyStringHere";
	
	key_exchange_client_->async_send_request(request,
        [this](rclcpp::Client<seal_msgs::srv::KeyExchange>::SharedFuture future_response) {
            auto response = future_response.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Key exchange successful: %s", response->message.c_str());
                send_ciphertext();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Key exchange failed: %s", response->message.c_str());
            }
        });
	
	RCLCPP_INFO(this->get_logger(), "sent key");
}

void SealClientNode::send_ciphertext() {
	std_msgs::msg::String msg;
	msg.data = "YourCiphertextHere";
	ciphertext_pub_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "Ciphertext sent");
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SealClientNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

