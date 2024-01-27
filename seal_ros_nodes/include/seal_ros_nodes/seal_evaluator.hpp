#ifndef SEAL_EVALUATOR_HPP
#define SEAL_EVALUATOR_HPP

#include "seal/seal.h"
#include "seal_ros_nodes/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class EvaluatorManager {
public:
	EvaluatorManager(std::vector<uint8_t> serialized_parms, 
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

#endif // SEAL_EVALUATOR_HPP

