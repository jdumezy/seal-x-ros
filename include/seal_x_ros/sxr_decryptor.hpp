#ifndef SXR_DECRYPTOR_HPP_
#define SXR_DECRYPTOR_HPP_

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

/**
 * @class SXRDecryptor
 * 
 * @brief A class used to handle decryption operations with serialized objects.
 * 
 * This class uses SEAL to decrypt serialized ciphertexts into floats. 
 * It is initialized using serialized parameters and a secret key for efficient 
 * and consistent decryption operations.
 * 
 * @see sxr_lib for the serialization and deserialization process.
 */
class SXRDecryptor {
public:
    /**
     * @brief Constructs a new SXRDecryptor object.
     *
     * Initializes the SEAL context and decryptor using the provided serialized parameters
     * and secret key.
     *
     * @param serializedParms Serialized SEAL encryption parameters.
     * @param SecretKey The secret key used for decryption.
     */
    SXRDecryptor(std::vector<uint8_t> serializedParms,
                 const seal::SecretKey &SecretKey);
    
    /**
     * @brief Decrypts a serialized ciphertext into a floating-point number.
     *
     * Decrypts and decodes the input serialized ciphertext using the SEAL library, 
     * returning the corresponding floating-point number.
     *
     * @param serializedCt Serialized ciphertext to be decrypted.
     * @return The decrypted floating-point number.
     */
    float decryptFloat(std::vector<uint8_t> serializedCt);
    std::vector<float> decryptFloatArray(std::vector<uint8_t> serializedCt);

private:
    std::shared_ptr<seal::SEALContext> mpContext;
    seal::Decryptor mDecryptor;
    seal::CKKSEncoder mEncoder;
    
    seal::Plaintext decrypt(std::vector<uint8_t> serializedCt);
};

#endif // SXR_DECRYPTOR_HPP_

