#ifndef SXR_EVALUATOR_HPP_
#define SXR_EVALUATOR_HPP_

#include "seal/seal.h"

#include "seal_x_ros/sxr_ciphertext.hpp"
#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

/**
 * @class SXREvaluator
 * 
 * @brief A class used for evaluating encrypted operations on serialized objects.
 * 
 * This class utilizes SEAL to perform operations on encrypted data, 
 * such as squaring, while working with serialized ciphertexts. 
 * It is initialized with serialized parameters, public key, 
 * relinearization keys, and Galois keys for efficient and secure operations.
 * 
 * @see sxr_lib for the serialization and deserialization process.
 */
class SXREvaluator {
public:
    /**
     * @brief Constructs a new SXREvaluator object.
     *
     * Initializes the SEAL context, encryptor, evaluator, and encoder using the provided 
     * serialized parameters, public key, relinearization keys, and Galois keys. 
     * The scale factor for encoding is also set during construction.
     *
     * @param serialized_parms Serialized SEAL encryption parameters.
     * @param serialized_pk Serialized public key for encryption.
     * @param serialized_rlk Serialized relinearization keys.
     * @param serialized_galk Serialized Galois keys.
     * @param scale The scale factor to be used in encoding and operations.
     */
    SXREvaluator(std::vector<uint8_t> serializedParms, 
                     std::vector<uint8_t> serializedPk, 
                     std::vector<uint8_t> serializedRlk,
                     std::vector<uint8_t> serializedGalk,
                     double scale);
    
    SXRCiphertext add(SXRCiphertext sxrctA, SXRCiphertext sxrctB);
    SXRCiphertext multiply(SXRCiphertext sxrctA, SXRCiphertext sxrctB);
    
    /**
     * @brief Performs the squaring operation on encrypted data.
     *
     * Accepts a serialized ciphertext, performs the squaring operation on it, 
     * and then returns the result as a serialized ciphertext.
     *
     * @param serialized_ct Serialized ciphertext to be squared.
     * @return Serialized ciphertext of the squared result.
     */
    SXRCiphertext square(SXRCiphertext sxrct);
    
    int matchDepth(SXRCiphertext& sxrctA, SXRCiphertext& sxrctB);

private:
    std::shared_ptr<seal::SEALContext> mpContext;
    seal::Encryptor mEncryptor;
    seal::CKKSEncoder mEncoder;
    seal::Evaluator mEvaluator;
    double mScale;
    seal::RelinKeys mRelinKeys;
    seal::GaloisKeys mGaloisKeys;
};

#endif // SXR_EVALUATOR_HPP_

