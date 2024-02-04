#ifndef SXR_EVALUATOR_HPP
#define SXR_EVALUATOR_HPP

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
	SXREvaluator(std::vector<uint8_t> serialized_parms, 
					 std::vector<uint8_t> serialized_pk, 
					 std::vector<uint8_t> serialized_rlk,
					 std::vector<uint8_t> serialized_galk,
					 double scale);
	
	SXRCiphertext add(SXRCiphertext sxrct_a, SXRCiphertext sxrct_b);
	SXRCiphertext multiply(SXRCiphertext sxrct_a, SXRCiphertext sxrct_b);
	
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

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Encryptor encryptor_;
	seal::CKKSEncoder encoder_;
	seal::Evaluator evaluator_;
	double scale_;
	seal::RelinKeys relin_keys_;
	seal::GaloisKeys galois_keys_;
};

#endif // SXR_EVALUATOR_HPP

