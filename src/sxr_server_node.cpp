#include "seal_x_ros/sxr_server_node.hpp"

SXRServerNode::SXRServerNode()
	: Node("sxr_server_node") {
	
	key_exchange_service_ = this->create_service<seal_x_ros::srv::KeyExchange>(
		"key_exchange_service",
		std::bind(&SXRServerNode::handle_key_exchange, this,
			std::placeholders::_1, std::placeholders::_2)
	);
	
	operation_request_service_ = this->create_service<seal_x_ros::srv::OperationRequest>(
		"operation_request_service",
		std::bind(&SXRServerNode::handle_operation_request, this,
			std::placeholders::_1, std::placeholders::_2)
	);
	server_message_client_ = this->create_client<seal_x_ros::srv::ServerMessage>(
		"server_message_service"
	);
}

void SXRServerNode::send_message() {	
	auto request = std::make_shared<seal_x_ros::srv::ServerMessage::Request>();
	
	float f = 3.1415f;
	std::vector<uint8_t> serialized_ct = encryptor_->encrypt_float(f);
	
	if (encryptor_) {
		request->serialized_ct = serialized_ct;
		
		server_message_client_->async_send_request(request,
			[this](rclcpp::Client<seal_x_ros::srv::ServerMessage>::SharedFuture future_response) {
				auto response = future_response.get();
				if (response->success) {
					RCLCPP_INFO(this->get_logger(), "Message sent successfully");
				}
				else {
					RCLCPP_ERROR(this->get_logger(), "Message sending failed");
				}
		});
	}
	else {
		RCLCPP_ERROR(this->get_logger(), "Encryptor is not initialized");
	}
	
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
	
	// Initialise SXR objects
	encryptor_.emplace(serialized_parms_, serialized_pk_, scale_);
	evaluator_.emplace(serialized_parms_, serialized_pk_, serialized_rlk_, serialized_galk_, scale_);
	
	// Initialise SEAL shared objects
	context_ = std::make_shared<seal::SEALContext>(deserialize_to_parms(serialized_parms_));
	
	public_key_ = std::make_shared<seal::PublicKey>(deserialize_to_pk(serialized_pk_, context_));
	relin_keys_ = std::make_shared<seal::RelinKeys>(deserialize_to_rlk(serialized_rlk_, context_));
	galois_keys_ = std::make_shared<seal::GaloisKeys>(deserialize_to_galk(serialized_galk_, context_));
	
	seal_encoder_ = std::make_shared<seal::CKKSEncoder>(*context_);
	seal_encryptor_ = std::make_shared<seal::Encryptor>(*context_, *public_key_);
	seal_evaluator_ = std::make_shared<seal::Evaluator>(*context_);
}

void SXRServerNode::handle_operation_request(const std::shared_ptr<seal_x_ros::srv::OperationRequest::Request> request,
	std::shared_ptr<seal_x_ros::srv::OperationRequest::Response> response) {
	
	RCLCPP_DEBUG(this->get_logger(), "Received ciphertext");
	
	if (evaluator_) {
		//TODO make shorter
		SXRCiphertext message1(serialized_parms_, serialized_pk_, serialized_rlk_, request->serialized_ct, scale_);
		RCLCPP_INFO(this->get_logger(), "message1: %d", message1.get_depth());
		
		SXRCiphertext message2(message1);
		RCLCPP_INFO(this->get_logger(), "message2: %d", message2.get_depth());
		
		SXRCiphertext message3(message1);
		RCLCPP_INFO(this->get_logger(), "message3: %d", message3.get_depth());
		
		SXRCiphertext message4(message1);
		RCLCPP_INFO(this->get_logger(), "message4: %d", message4.get_depth());
		
		
		SXRCiphertext result1 = evaluator_->multiply(message1, message2);
		RCLCPP_INFO(this->get_logger(), "result1: %d", result1.get_depth());
		
		RCLCPP_INFO(this->get_logger(), "match?: %d %d", result1.get_depth(), message1.get_depth());
		SXRCiphertext result2 = evaluator_->multiply(result1, message1);
		RCLCPP_INFO(this->get_logger(), "result2: %d", result2.get_depth());
		
		RCLCPP_INFO(this->get_logger(), "match?: %d %d", result2.get_depth(), message3.get_depth());
		SXRCiphertext result3 = evaluator_->add(result2, message3);
		RCLCPP_INFO(this->get_logger(), "result3: %d", result3.get_depth());
		
		RCLCPP_INFO(this->get_logger(), "match?: %d %d", result3.get_depth(), message4.get_depth());
		SXRCiphertext result4 = evaluator_->multiply(result3, message4);
		RCLCPP_INFO(this->get_logger(), "result4: %d", result4.get_depth());
		
		SXRCiphertext result5 = evaluator_->multiply(result4, result3);
		RCLCPP_INFO(this->get_logger(), "result5: %d", result4.get_depth());
		
		RCLCPP_DEBUG(this->get_logger(), "Sending result ciphertext...");
		
		response->success = true;
		response->serialized_ct_res = serialize_seal_object(result5.get_ct());
	}
	else {
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

