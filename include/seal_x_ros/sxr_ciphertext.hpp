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
    std::shared_ptr<seal::Encryptor> encryptor_;
    std::shared_ptr<seal::CKKSEncoder> encoder_;
    std::shared_ptr<seal::Evaluator> evaluator_;
    seal::Ciphertext ciphertext_;
    std::shared_ptr<seal::RelinKeys> relin_keys_;
    std::shared_ptr<seal::PublicKey> public_key_;
    double scale_;
    int depth_;
    std::vector<double> cm_prime_array_;
};

#endif // SXR_CIPHERTEXT_HPP

