#include "seal_x_ros/sxr_encryptor.hpp"

SXREncryptor::SXREncryptor(std::vector<uint8_t> serialized_parms, 
                           std::vector<uint8_t> serialized_pk, 
                           double scale)
    : context_(pContextFromParms(serialized_parms)),
      encryptor_(*context_, deserializeToPk(serialized_pk, context_)), 
      encoder_(*context_), scale_(scale) {
}

std::vector<uint8_t> SXREncryptor::encrypt(seal::Plaintext encoded_pt) {
    seal::Ciphertext encrypted_ct;
    encryptor_.encrypt(encoded_pt, encrypted_ct);
    
    std::vector<uint8_t> serialized_ct = serializeSealObject(encrypted_ct);
    
    return serialized_ct;
}

//TODO add srx ciphertext
std::vector<uint8_t> SXREncryptor::encrypt_float(float input_float) {
    seal::Plaintext encoded_pt;
    encoder_.encode(input_float, scale_, encoded_pt);
    
    return encrypt(encoded_pt);
}

//TODO add srx ciphertext
std::vector<uint8_t> SXREncryptor::encrypt_float_array(const std::vector<float>& input_float_array) {
    std::vector<double> double_array = floatArrayToDoubleArray(input_float_array);
    
    size_t slot_count = encoder_.slot_count();
    if (input_float_array.size() > slot_count / 2) {
        throw std::runtime_error("Input array is too large for CKKS slots");
    }
    
    seal::Plaintext encoded_pt;
    encoder_.encode(double_array, scale_, encoded_pt);
    
    return encrypt(encoded_pt);
}

