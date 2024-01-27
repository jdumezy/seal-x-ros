#ifndef SEAL_SERVER_NODE_HPP
#define SEAL_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "seal_ros_nodes/seal_evaluator.hpp"
#include "seal_ros_nodes/sxr_lib.hpp"

#include "seal/seal.h"

#include "seal_msgs/srv/key_exchange.hpp"
#include "seal_msgs/srv/operation_request.hpp"
//#include "seal_ros_nodes/srv/key_exchange.hpp"
//#include "seal_ros_nodes/srv/operation_request.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class SealServerNode : public rclcpp::Node {
public:
	SealServerNode();

private:
	void handle_key_exchange(const std::shared_ptr<seal_msgs::srv::KeyExchange::Request> request, std::shared_ptr<seal_msgs::srv::KeyExchange::Response> response);
	void handle_operation_request(const std::shared_ptr<seal_msgs::srv::OperationRequest::Request> request, std::shared_ptr<seal_msgs::srv::OperationRequest::Response> response);

	rclcpp::Service<seal_msgs::srv::KeyExchange>::SharedPtr key_exchange_service_;
	rclcpp::Service<seal_msgs::srv::OperationRequest>::SharedPtr operation_request_service_;
	
	std::vector<uint8_t> serialized_parms_;
	std::vector<uint8_t> serialized_pk_;
	std::vector<uint8_t> serialized_rlk_;
	std::vector<uint8_t> serialized_galk_;
	double scale_;
	
	std::optional<EvaluatorManager> evaluator_;
};

#endif // SEAL_SERVER_NODE_HPP

