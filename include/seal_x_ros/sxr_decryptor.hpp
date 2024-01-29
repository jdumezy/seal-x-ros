#ifndef SXR_DECRYPTOR_HPP
#define SXR_DECRYPTOR_HPP

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
	 * @param serialized_parms Serialized SEAL encryption parameters.
	 * @param secret_key The secret key used for decryption.
	 */
	SXRDecryptor(std::vector<uint8_t> serialized_parms,
				 const seal::SecretKey &secret_key);
	
	/**
	 * @brief Decrypts a serialized ciphertext into a floating-point number.
	 *
	 * Decrypts and decodes the input serialized ciphertext using the SEAL library, 
	 * returning the corresponding floating-point number.
	 *
	 * @param encrypted_tt Serialized ciphertext to be decrypted.
	 * @return The decrypted floating-point number.
	 */
	float decrypt_float(std::vector<uint8_t> encrypted_tt);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Decryptor decryptor_;
	seal::CKKSEncoder encoder_;
};

#endif // SXR_DECRYPTOR_HPP

