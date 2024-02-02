#ifndef SXR_CIPHERTEXT_HPP
#define SXR_CIPHERTEXT_HPP

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>
#include <stdexcept>

class SXRCiphertext {
public:
	SXRCiphertext(std::vector<uint8_t> serialized_parms,
				  std::vector<uint8_t> serialized_pk,
				  std::vector<uint8_t> serialized_rlk,
				  std::vector<uint8_t> serialized_ct,
				  double scale);
	
	seal::Ciphertext get_ct();
	int get_depth();
	void set_depth(int new_depth);
	void set_scale(int new_scale);
	void match_depth(int new_depth);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Ciphertext ciphertext_;
	seal::Encryptor encryptor_;
	seal::CKKSEncoder encoder_;
	seal::Evaluator evaluator_;
	seal::RelinKeys relin_keys_;
	int depth_;
	double scale_;
	std::vector<double> cm_prime_array_;
};

#endif // SXR_CIPHERTEXT_HPP

