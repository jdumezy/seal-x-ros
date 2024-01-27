#include "seal_ros_nodes/seal_client_node.hpp"

SealClientNode::SealClientNode()
	: rclcpp::Node("seal_client_node"),
	  parmsAndKeys_(), 
	  encryptor_(parmsAndKeys_.get_serialized_parms(), parmsAndKeys_.get_serialized_pk(), parmsAndKeys_.get_scale()),
	  decryptor_(parmsAndKeys_.get_serialized_parms(), parmsAndKeys_.get_secret_key()) {
	
	// Create client, publisher and subscriber
	key_exchange_client_ = this->create_client<seal_msgs::srv::KeyExchange>(
		"key_exchange_service"
	);
	ciphertext_pub_ = this->create_publisher<std_msgs::msg::String>(
		"seal_ciphertext_topic", 10
	);
	response_sub_ = this->create_subscription<std_msgs::msg::String>(
		"response_topic", 10,
		std::bind(&SealClientNode::response_callback, this, std::placeholders::_1)
	);
	
	parms_ = parmsAndKeys_.get_serialized_parms();
	public_key_ = parmsAndKeys_.get_serialized_pk();
	relin_keys_ = parmsAndKeys_.get_serialized_rlk();
	secret_key_ = parmsAndKeys_.get_secret_key();
	scale_ = parmsAndKeys_.get_scale();
	
	connection_and_send_key();
}

// Send key and context to server
void SealClientNode::connection_and_send_key() {
	while (!key_exchange_client_->wait_for_service(std::chrono::seconds(1))) {
		RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
	}
	
	auto request = std::make_shared<seal_msgs::srv::KeyExchange::Request>();
	request->parms = parms_;
	request->public_key = public_key_;
	request->relin_keys = relin_keys_;
	
	key_exchange_client_->async_send_request(request,
		[this](rclcpp::Client<seal_msgs::srv::KeyExchange>::SharedFuture future_response) {
			auto response = future_response.get();
			if (response->success) {
				RCLCPP_INFO(this->get_logger(), "Key exchange successful");
				send_ciphertext();
			}
			else {
				RCLCPP_ERROR(this->get_logger(), "Key exchange failed");
			}
	});
}

void SealClientNode::send_ciphertext() {
	std_msgs::msg::String msg;
	msg.data = "YourCiphertextHere";
	ciphertext_pub_->publish(msg);
	RCLCPP_DEBUG(this->get_logger(), "Ciphertext sent");
}

void SealClientNode::response_callback(const std_msgs::msg::String::SharedPtr msg) {
	RCLCPP_INFO(this->get_logger(), "Received response");
	const std_msgs::msg::String::SharedPtr m = msg;
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SealClientNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

