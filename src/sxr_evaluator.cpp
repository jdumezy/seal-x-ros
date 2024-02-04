#include "seal_x_ros/sxr_evaluator.hpp"

SXREvaluator::SXREvaluator(std::vector<uint8_t> serialized_parms, 
								   std::vector<uint8_t> serialized_pk,
								   std::vector<uint8_t> serialized_rlk,
								   std::vector<uint8_t> serialized_galk, 
								   double scale)
	: context_(context_from_parms(serialized_parms)),
	  encryptor_(*context_, deserialize_to_pk(serialized_pk, context_)),
	  encoder_(*context_), evaluator_(*context_), scale_(scale),
	  relin_keys_(deserialize_to_rlk(serialized_rlk, context_)),
	  galois_keys_(deserialize_to_galk(serialized_galk, context_)){
}

SXRCiphertext SXREvaluator::add(SXRCiphertext sxrct_a, SXRCiphertext sxrct_b) {
	int depth = std::max(sxrct_a.get_depth(), sxrct_b.get_depth());
	sxrct_a.match_depth(depth);
	sxrct_b.match_depth(depth);
	
	seal::Ciphertext result_ct;
	
	evaluator_.add(sxrct_a.get_ct(), sxrct_b.get_ct(), result_ct);
	
	SXRCiphertext result(sxrct_a);
	result.set_ct(result_ct);
	
	return result;
}

SXRCiphertext SXREvaluator::multiply(SXRCiphertext sxrct_a, SXRCiphertext sxrct_b) {
	int depth = std::max(sxrct_a.get_depth(), sxrct_b.get_depth());
	sxrct_a.match_depth(depth);
	sxrct_b.match_depth(depth);
	
	seal::Ciphertext result_ct;
	
	evaluator_.multiply(sxrct_a.get_ct(), sxrct_b.get_ct(), result_ct);
	evaluator_.relinearize_inplace(result_ct, relin_keys_);
	evaluator_.rescale_to_next_inplace(result_ct);
	
	SXRCiphertext result(sxrct_a);
	result.set_ct(result_ct);
	result.set_depth(depth + 1);
	
	return result;
}

SXRCiphertext SXREvaluator::square(SXRCiphertext sxrct) {
	seal::Ciphertext result_ct;
	
	evaluator_.square(sxrct.get_ct(), result_ct);
	evaluator_.relinearize_inplace(result_ct, relin_keys_);
	evaluator_.rescale_to_next_inplace(result_ct);
	
	SXRCiphertext result(sxrct);
	result.set_ct(result_ct);
	result.set_depth(sxrct.get_depth() + 1);
	return result;
}

