#include "seal_ros_nodes/seal_server_node.hpp"

SealServerNode::SealServerNode()
	: Node("seal_server_node") {
	
	key_exchange_service_ = this->create_service<seal_msgs::srv::KeyExchange>("key_exchange_service",
		std::bind(&SealServerNode::handle_key_exchange, this,
		std::placeholders::_1, std::placeholders::_2)
	);

	ciphertext_sub_ = this->create_subscription<std_msgs::msg::String>(
		"seal_ciphertext_topic",
		10,
		std::bind(&SealServerNode::ciphertext_callback, this, std::placeholders::_1)
	);
	
	response_pub_ = this->create_publisher<std_msgs::msg::String>("response_topic", 10);
}

void SealServerNode::handle_key_exchange(const std::shared_ptr<seal_msgs::srv::KeyExchange::Request> request,
	std::shared_ptr<seal_msgs::srv::KeyExchange::Response> response) {
	
	parms_ = request->parms;
	public_key_ = request->public_key;
	relin_keys_ = request->relin_keys;
	
	RCLCPP_INFO(this->get_logger(), "Received public key");
	
	response->success = true;
	response->message = "Public key received successfully";
}

void SealServerNode::ciphertext_callback(const std_msgs::msg::String::SharedPtr msg) {
	RCLCPP_DEBUG(this->get_logger(), "Received ciphertext");
	
	std_msgs::msg::String response_msg;
	response_msg.data = "Response to received ciphertext";
	response_pub_->publish(response_msg);
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SealServerNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

