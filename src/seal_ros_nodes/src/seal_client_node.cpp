#include "seal_x_ros/seal_client_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "seal_msgs/msg/seal_data.hpp"
#include "seal/seal.h"
#include <iostream>
#include <sstream>

using namespace rclcpp;
using namespace std;
using namespace seal;

SealClientNode::SealClientNode() : Node("seal_client_node") {
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
    // Set up SEAL environment (your original SEAL code)
    EncryptionParameters parms(scheme_type::bfv);
    size_t poly_modulus_degree = 4096;
    parms.set_poly_modulus_degree(poly_modulus_degree);
    parms.set_coeff_modulus(CoeffModulus::BFVDefault(poly_modulus_degree));
    parms.set_plain_modulus(1024);
    SEALContext context(parms);

    // Perform a simple SEAL operation (your original SEAL code)
    KeyGenerator keygen(context);
    SecretKey secret_key = keygen.secret_key();
    PublicKey public_key;
    keygen.create_public_key(public_key);

    Encryptor encryptor(context, public_key);
    Evaluator evaluator(context);
    Decryptor decryptor(context, secret_key);

    Plaintext plain("1x^2 + 2x^1 + 3");
    Ciphertext encrypted;
    encryptor.encrypt(plain, encrypted);

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

