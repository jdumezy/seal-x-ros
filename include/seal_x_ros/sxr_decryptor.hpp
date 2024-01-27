#ifndef SXR_DECRYPTOR_HPP
#define SXR_DECRYPTOR_HPP

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class SXRDecryptor {
public:
	SXRDecryptor(std::vector<uint8_t> serialized_parms,
				 const seal::SecretKey &secret_key);

	float decrypt_float(std::vector<uint8_t> encrypted_tt);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Decryptor decryptor_;
	seal::CKKSEncoder encoder_;
};

#endif // SXR_DECRYPTOR_HPP

