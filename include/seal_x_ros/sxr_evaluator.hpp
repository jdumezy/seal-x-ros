#ifndef SXR_EVALUATOR_HPP
#define SXR_EVALUATOR_HPP

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class SXREvaluator {
public:
	SXREvaluator(std::vector<uint8_t> serialized_parms, 
					 std::vector<uint8_t> serialized_pk, 
					 std::vector<uint8_t> serialized_rlk,
					 std::vector<uint8_t> serialized_galk,
					 double scale);

	std::vector<uint8_t> square(std::vector<uint8_t> serialized_ct);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Encryptor encryptor_;
	seal::CKKSEncoder encoder_;
	seal::Evaluator evaluator_;
	seal::RelinKeys relin_keys_;
	seal::GaloisKeys galois_keys_;
	double scale_;
};

#endif // SXR_EVALUATOR_HPP

