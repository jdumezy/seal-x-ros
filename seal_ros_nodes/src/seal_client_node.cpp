#include "seal_ros_nodes/seal_client_node.hpp"
#include "seal_ros_nodes/seal_encryptor.hpp"
#include "seal_ros_nodes/seal_context.hpp"
#include "seal_msgs/msg/seal_data.hpp"
#include "seal/seal.h"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <sstream>
#include <vector>

using namespace rclcpp;
using namespace std;
using namespace seal;

SealClientNode::SealClientNode() : Node("seal_client_node") {
    std::string config_file;
    this->declare_parameter<std::string>("config_file", "config/encryption_params_bfv.yaml");
    this->get_parameter("config_file", config_file);

    ConfigurationManager config_manager(config_file);
    auto parms = config_manager.get_encryption_parameters(); // Retrieve EncryptionParameters

    // Create SEALContext from EncryptionParameters
    seal::SEALContext context(parms);

    // Key generation
    seal::KeyGenerator keygen(context);
    seal::PublicKey public_key = keygen.public_key();
    // SecretKey secret_key = keygen.secret_key(); // Use if needed

    // Assuming Encryptor is correctly defined to take these parameters
    encryptor_ = std::make_unique<Encryptor>(context, public_key);

    // Publisher for encrypted data
    encrypted_publisher = this->create_publisher<seal_msgs::msg::SealData>("encrypted_topic", 10);

    // Subscriber for decrypted data
    decrypted_subscriber = this->create_subscription<std_msgs::msg::String>(
        "decrypted_topic", 10, bind(&SealClientNode::decrypted_callback, this, placeholders::_1));

    // Perform encryption and publish
    test_seal_function();
}

void SealClientNode::decrypted_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received response: '%s'", msg->data.c_str());
}

void SealClientNode::test_seal_function() {
    // Example plaintext
    Plaintext plain("1x^2 + 2x^1 + 3");

    // Encrypt the plaintext
    Ciphertext encrypted = encryptor_->encrypt(plain);

    // Serialize and publish the encrypted data
    std::stringstream ss;
    encrypted.save(ss);
    auto encrypted_msg = seal_msgs::msg::SealData();
    encrypted_msg.data = std::vector<uint8_t>(std::istreambuf_iterator<char>(ss), {});

    encrypted_publisher->publish(encrypted_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SealClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


