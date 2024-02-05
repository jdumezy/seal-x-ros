#include "seal_x_ros/sxr_ciphertext.hpp"

SXRCiphertext::SXRCiphertext(seal::Ciphertext ciphertext)
	: ciphertext_(ciphertext) {
	depth_ = 0;
	
//	auto& contextData = context_->key_context_data();
//	auto coeffModulus = contextData.parms().coeff_modulus();
//	size_t n = coeffModulus.size();
//	
//	for (int i = n - 2; i >= 1; i--) {
//		primeArray_.push_back(static_cast<double>(coeffModulus[i].value()));
//	}
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
