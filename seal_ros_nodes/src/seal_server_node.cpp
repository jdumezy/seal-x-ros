#include "seal_x_ros/seal_server_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "seal_msgs/msg/seal_data.hpp"
#include <memory>

using namespace std;
using namespace rclcpp;

SealServerNode::SealServerNode() : Node("seal_server_node") {
    // Subscriber for encrypted data
    encrypted_subscriber = this->create_subscription<seal_msgs::msg::SealData>(
        "encrypted_topic", 10, bind(&SealServerNode::encrypted_callback, this, placeholders::_1));

    // Publisher for decrypted or processed data
    decrypted_publisher = this->create_publisher<std_msgs::msg::String>("decrypted_topic", 10);
}

void SealServerNode::encrypted_callback(const seal_msgs::msg::SealData::SharedPtr /*msg*/) {
    // Example processing: Log received message
    RCLCPP_INFO(this->get_logger(), "Received encrypted data");

    // Process and publish a response (for simplicity, just a text message here)
    auto response = std_msgs::msg::String();
    response.data = "Processed data";
    decrypted_publisher->publish(response);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<SealServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

