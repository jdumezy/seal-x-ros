#ifndef SXR_SERVER_NODE_HPP
#define SXR_SERVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "seal/seal.h"

#include "seal_x_ros/sxr_evaluator.hpp"
#include "seal_x_ros/sxr_lib.hpp"

#include "seal_x_ros/srv/key_exchange.hpp"
#include "seal_x_ros/srv/operation_request.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class SXRServerNode : public rclcpp::Node {
public:
	SXRServerNode();

private:
	void handle_key_exchange(const std::shared_ptr<seal_x_ros::srv::KeyExchange::Request> request, std::shared_ptr<seal_x_ros::srv::KeyExchange::Response> response);
	void handle_operation_request(const std::shared_ptr<seal_x_ros::srv::OperationRequest::Request> request, std::shared_ptr<seal_x_ros::srv::OperationRequest::Response> response);

	rclcpp::Service<seal_x_ros::srv::KeyExchange>::SharedPtr key_exchange_service_;
	rclcpp::Service<seal_x_ros::srv::OperationRequest>::SharedPtr operation_request_service_;
	
	std::vector<uint8_t> serialized_parms_;
	std::vector<uint8_t> serialized_pk_;
	std::vector<uint8_t> serialized_rlk_;
	std::vector<uint8_t> serialized_galk_;
	double scale_;
	
	std::optional<SXREvaluator> evaluator_;
};

#endif // SXR_SERVER_NODE_HPP

