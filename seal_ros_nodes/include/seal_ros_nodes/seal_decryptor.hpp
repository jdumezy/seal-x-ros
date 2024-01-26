#ifndef SEAL_DECRYPTOR_HPP
#define SEAL_DECRYPTOR_HPP

#include "seal/seal.h"
#include "seal_ros_nodes/sxr_lib.hpp"

#include <vector>
#include <memory>

class DecryptorManager {
public:
	DecryptorManager(std::vector<uint8_t> serialized_parms,
					 const seal::SecretKey &secret_key);

	float decrypt_float(seal::Ciphertext encryptedCiphertext);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Decryptor decryptor_;
	seal::CKKSEncoder encoder_;
};

#endif // SEAL_DECRYPTOR_HPP

