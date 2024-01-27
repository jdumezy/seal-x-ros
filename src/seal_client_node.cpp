#include "seal_x_ros/seal_client_node.hpp"

SealClientNode::SealClientNode()
	: rclcpp::Node("seal_client_node"),
	  parmsAndKeys_(), 
	  encryptor_(parmsAndKeys_.get_serialized_parms(), parmsAndKeys_.get_serialized_pk(), parmsAndKeys_.get_scale()),
	  decryptor_(parmsAndKeys_.get_serialized_parms(), parmsAndKeys_.get_secret_key()) {
	
	// Create client, publisher and subscriber
	key_exchange_client_ = this->create_client<seal_x_ros::srv::KeyExchange>(
		"key_exchange_service"
	);
	operation_request_client_ = this->create_client<seal_x_ros::srv::OperationRequest>(
		"operation_request_service"
	);
	
	serialized_parms_ = parmsAndKeys_.get_serialized_parms();
	serialized_pk_ = parmsAndKeys_.get_serialized_pk();
	serialized_rlk_ = parmsAndKeys_.get_serialized_rlk();
	serialized_galk_ = parmsAndKeys_.get_serialized_galk();
	secret_key_ = parmsAndKeys_.get_secret_key();
	scale_ = parmsAndKeys_.get_scale();
	
	connection_and_send_key();
}

// Send key and context to server
void SealClientNode::connection_and_send_key() {
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
			}
			else {
				RCLCPP_ERROR(this->get_logger(), "Key exchange failed");
			}
	});
}

void SealClientNode::send_ciphertext() {
	float f = 3.1415f;
	std::vector<uint8_t> serialized_ct = encryptor_.encrypt_float(f);
	
	RCLCPP_DEBUG(this->get_logger(), "Sending ciphertext: %f", f);
	
	auto request = std::make_shared<seal_x_ros::srv::OperationRequest::Request>();
	request->serialized_ct = serialized_ct;
	
	operation_request_client_->async_send_request(request,
		[this](rclcpp::Client<seal_x_ros::srv::OperationRequest>::SharedFuture future_response) {
			
			auto response = future_response.get();
			if (response->success) {
				RCLCPP_DEBUG(this->get_logger(), "Ciphertext received");
				std::vector<uint8_t> serialized_ct_res = response->serialized_ct_res;
				float result = decryptor_.decrypt_float(serialized_ct_res);
				RCLCPP_INFO(this->get_logger(), "Received response: %f", result);
			}
			else {
				RCLCPP_ERROR(this->get_logger(), "Failed to send ciphertext");
			}
	});
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SealClientNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

