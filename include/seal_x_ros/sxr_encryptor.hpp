#ifndef SXR_ENCRYPTOR_HPP
#define SXR_ENCRYPTOR_HPP

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class SXREncryptor {
public:
	SXREncryptor(std::vector<uint8_t> serialized_parms, 
				 std::vector<uint8_t> serialized_pk, 
				 double scale);

	std::vector<uint8_t> encrypt_float(float input_float);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Encryptor encryptor_;
	seal::CKKSEncoder encoder_;
	double scale_;
};

#endif // SXR_ENCRYPTOR_HPP

