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
	SXRCiphertext(const SXRCiphertext& other);
	
	seal::Ciphertext get_ct();
	int get_depth();
	void set_ct(seal::Ciphertext new_ct);
	void set_depth(int new_depth);
	void match_depth(int new_depth);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Encryptor encryptor_;
	seal::CKKSEncoder encoder_;
	seal::Evaluator evaluator_;
	seal::Ciphertext ciphertext_;
	seal::RelinKeys relin_keys_;
	seal::PublicKey public_key_;
	double scale_;
	int depth_;
	std::vector<double> cm_prime_array_;
};

#endif // SXR_CIPHERTEXT_HPP

