#include "seal_x_ros/sxr_server_node.hpp"

SXRServerNode::SXRServerNode()
	: Node("sxr_server_node") {
	
	key_exchange_service_ = this->create_service<seal_x_ros::srv::KeyExchange>("key_exchange_service",
		std::bind(&SXRServerNode::handle_key_exchange, this,
			std::placeholders::_1, std::placeholders::_2)
	);
	
	operation_request_service_ = this->create_service<seal_x_ros::srv::OperationRequest>(
		"operation_request_service",
		std::bind(&SXRServerNode::handle_operation_request, this,
			std::placeholders::_1, std::placeholders::_2)
	);
}

void SXRServerNode::handle_key_exchange(const std::shared_ptr<seal_x_ros::srv::KeyExchange::Request> request,
	std::shared_ptr<seal_x_ros::srv::KeyExchange::Response> response) {
	
	serialized_parms_ = request->serialized_parms;
	serialized_pk_ = request->serialized_pk;
	serialized_rlk_ = request->serialized_rlk;
	serialized_galk_ = request->serialized_galk;
	scale_ = request->scale;
	
	RCLCPP_INFO(this->get_logger(), "Received public key");
	
	response->success = true;
	response->message = "Public key received successfully";
	
	evaluator_.emplace(serialized_parms_, serialized_pk_, serialized_rlk_, serialized_galk_, scale_);
}

void SXRServerNode::handle_operation_request(const std::shared_ptr<seal_x_ros::srv::OperationRequest::Request> request,
	std::shared_ptr<seal_x_ros::srv::OperationRequest::Response> response) {
	
	RCLCPP_DEBUG(this->get_logger(), "Received ciphertext");
	
	if (evaluator_) {
		std::vector<uint8_t> result = evaluator_->square(request->serialized_ct);
		
		RCLCPP_DEBUG(this->get_logger(), "Sending result ciphertext...");
		
		response->success = true;
		response->serialized_ct_res = result;
	}
	else {
		RCLCPP_ERROR(this->get_logger(), "Evaluator is not initialized");
		response->success = false;
	}
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SXRServerNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
