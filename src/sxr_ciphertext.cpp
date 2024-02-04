#include "seal_x_ros/sxr_ciphertext.hpp"

SXRCiphertext::SXRCiphertext(std::vector<uint8_t> serialized_parms,
							 std::vector<uint8_t> serialized_pk,
							 std::vector<uint8_t> serialized_rlk,
							 std::vector<uint8_t> serialized_ct,
							 double scale)
	: context_(context_from_parms(serialized_parms)),
	  public_key_(std::make_shared<seal::PublicKey>(deserialize_to_pk(serialized_pk, context_))),
	  scale_(scale) {
	encryptor_ = std::make_shared<seal::Encryptor>(*context_, *public_key_);
	encoder_ = std::make_shared<seal::CKKSEncoder>(*context_);
	evaluator_ = std::make_shared<seal::Evaluator>(*context_);
	ciphertext_ = deserialize_to_ct(serialized_ct, context_);
	relin_keys_ = std::make_shared<seal::RelinKeys>(deserialize_to_rlk(serialized_rlk, context_));
	depth_ = 0;
	
	auto& context_data = *context_->key_context_data();
	auto coeff_modulus = context_data.parms().coeff_modulus();
	size_t n = coeff_modulus.size();
	
	for (int i = n - 2; i >= 1; i--) {
		cm_prime_array_.push_back(static_cast<double>(coeff_modulus[i].value()));
	}
}

SXRCiphertext::SXRCiphertext(const SXRCiphertext& other)
	: context_(other.context_),
	  encryptor_(std::make_shared<seal::Encryptor>(*other.context_, *other.public_key_)),
	  encoder_(other.encoder_),
	  evaluator_(other.evaluator_),
	  ciphertext_(other.ciphertext_),
	  relin_keys_(other.relin_keys_),
	  public_key_(other.public_key_),
	  scale_(other.scale_),
	  depth_(other.depth_),
	  cm_prime_array_(other.cm_prime_array_) {
}

seal::Ciphertext SXRCiphertext::get_ct() {
	return ciphertext_;
}

int SXRCiphertext::get_depth() {
	return depth_;
}

void SXRCiphertext::set_ct(seal::Ciphertext new_ct) {
	ciphertext_ = new_ct;
}

void SXRCiphertext::set_depth(int new_depth) {
	depth_ = new_depth;
}

void SXRCiphertext::match_depth(int new_depth) {
	int depth_diff = new_depth - depth_;
	
	if (depth_diff > 0) {
		float one = 1.0f;

		seal::Plaintext one_pt;
		(*encoder_).encode(one, scale_, one_pt);
		
		seal::Ciphertext one_ct;
		(*encryptor_).encrypt(one_pt, one_ct);
		
		seal::Ciphertext new_ciphertext = ciphertext_;
		
		for (int i = 0; i < depth_diff; i++) {
			if (i >= depth_) {
				(*evaluator_).multiply_inplace(new_ciphertext, one_ct);
				(*evaluator_).relinearize_inplace(new_ciphertext, *relin_keys_);
				(*evaluator_).rescale_to_next_inplace(new_ciphertext);
			}
			
			(*evaluator_).square_inplace(one_ct);
			(*evaluator_).relinearize_inplace(one_ct, *relin_keys_);
			(*evaluator_).rescale_to_next_inplace(one_ct);
		}
		
		ciphertext_ = new_ciphertext;
		depth_ = new_depth;
	}
}
