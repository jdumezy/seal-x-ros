#include "seal_x_ros/sxr_ciphertext.hpp"

SXRCiphertext::SXRCiphertext(std::vector<uint8_t> serialized_parms,
							 std::vector<uint8_t> serialized_pk,
							 std::vector<uint8_t> serialized_rlk,
							 std::vector<uint8_t> serialized_ct,
							 double scale)
	: context_(context_from_parms(serialized_parms)),
	  encryptor_(*context_, deserialize_to_pk(serialized_pk, context_)), 
	  encoder_(*context_), evaluator_(*context_),
	  ciphertext_(deserialize_to_ct(serialized_ct, context_)),
	  relin_keys_(deserialize_to_rlk(serialized_rlk, context_)),
	  scale_(scale){
	
	depth_ = 0;
	
	auto& context_data = *context_->key_context_data();
	auto coeff_modulus = context_data.parms().coeff_modulus();
	size_t n = coeff_modulus.size();
	
	std::vector<double> cm_prime_array_;
	
	for (int i = n - 2; i >= 1; i--) {
		cm_prime_array_.push_back(static_cast<double>(coeff_modulus[i].value()));
	}
}

seal::Ciphertext SXRCiphertext::get_ct() {
	return ciphertext_;
}

int SXRCiphertext::get_depth() {
	return depth_;
}

void SXRCiphertext::set_depth(int new_depth) {
	depth_ = new_depth;
}

void SXRCiphertext::set_scale(int new_scale) {
	scale_ = new_scale;
}

void SXRCiphertext::match_depth(int new_depth) {
	int depth_diff = new_depth - depth_;
	
	if (depth_diff < 0) {
		throw std::invalid_argument("New depth must be bigger than current depth");
	}
	else if (depth_diff > 0) {
		float one = 1.0f;

		seal::Plaintext one_pt;
		encoder_.encode(one, scale_, one_pt);
		
		seal::Ciphertext one_ct;
		encryptor_.encrypt(one_pt, one_ct);
		
		for (int i = 0; i < depth_diff - 1; i++) {
			evaluator_.square_inplace(one_ct);
			evaluator_.relinearize_inplace(one_ct, relin_keys_);
			evaluator_.rescale_to_next_inplace(one_ct);
		}
		
		seal::Ciphertext new_ciphertext;
		evaluator_.multiply(ciphertext_, one_ct, new_ciphertext);
		evaluator_.relinearize_inplace(new_ciphertext, relin_keys_);
		evaluator_.rescale_to_next_inplace(new_ciphertext);
		
		ciphertext_ = new_ciphertext;
		depth_ = new_depth;
	}
}

