#ifndef SEAL_ENCRYPTOR_HPP
#define SEAL_ENCRYPTOR_HPP

#include "seal/seal.h"
#include "seal_ros_nodes/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class EncryptorManager {
public:
	EncryptorManager(std::vector<uint8_t> serialized_parms, 
					 std::vector<uint8_t> serialized_pk, 
					 double scale);

	std::vector<uint8_t> encrypt_float(float inputFloat);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Encryptor encryptor_;
	seal::CKKSEncoder encoder_;
	double scale_;
};

#endif // SEAL_ENCRYPTOR_HPP

