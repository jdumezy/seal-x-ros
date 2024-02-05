#ifndef SXR_ENCRYPTOR_HPP_
#define SXR_ENCRYPTOR_HPP_

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

/**
 * @class SXREncryptor
 * 
 * @brief A class used to handle encryption operations with serialized objects.
 * 
 * This class uses SEAL to encrypt floats into serialized ciphertext.
 * It is initialized using serialized parameters for efficiency and consistency.
 * 
 * @see sxr_lib for the serialization and deserialization process.
 */
class SXREncryptor {
public:
    /**
     * @brief Constructs a new SXREncryptor object.
     *
     * Initializes the SEAL context, encryptor, and encoder using the provided serialized parameters
     * and public key. The scale factor for encoding is also set during construction.
     *
     * @param serializedParms Serialized SEAL encryption parameters.
     * @param serializedPk Serialized public key for encryption.
     * @param scale The scale factor to be used in encoding floating-point numbers.
     */
    SXREncryptor(std::vector<uint8_t> serializedParms, 
                 std::vector<uint8_t> serializedPk, 
                 double scale);
    
    /**
     * @brief Encrypts a single floating-point number.
     *
     * Encodes and encrypts the input float using the SEAL library. The encrypted data is then 
     * serialized into a vector of bytes for transmission.
     *
     * @param inputFloat The floating-point number to be encrypted.
     * @return Serialized ciphertext of the encrypted floating-point number.
     */
    std::vector<uint8_t> encryptFloat(float inputFloat);
    std::vector<uint8_t> encryptFloatArray(const std::vector<float>& inputFloatArray);

private:
    std::shared_ptr<seal::SEALContext> mpContext;
    seal::Encryptor mEncryptor;
    seal::CKKSEncoder mEncoder;
    double mScale;
    
    std::vector<uint8_t> encrypt(seal::Plaintext encodedPlaintext);
};

#endif // SXR_ENCRYPTOR_HPP_

