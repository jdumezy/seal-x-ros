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
//TODO add init
SXRCiphertext SXREvaluator::add(SXRCiphertext sxrct_a, SXRCiphertext sxrct_b) {
	int depth = match_depth(sxrct_a, sxrct_b);
	
	seal::Ciphertext result_ct;
	
	evaluator_.add(sxrct_a.get_ct(), sxrct_b.get_ct(), result_ct);
	
	SXRCiphertext result(result_ct);
	result.set_depth(depth);
	
	return result;
}

SXRCiphertext SXREvaluator::multiply(SXRCiphertext sxrct_a, SXRCiphertext sxrct_b) {
	int depth = match_depth(sxrct_a, sxrct_b);
	
	seal::Ciphertext result_ct;
	
	evaluator_.multiply(sxrct_a.get_ct(), sxrct_b.get_ct(), result_ct);
	evaluator_.relinearize_inplace(result_ct, relin_keys_);
	evaluator_.rescale_to_next_inplace(result_ct);
	
	SXRCiphertext result(result_ct);
	result.set_depth(depth + 1);
	
	return result;
}

SXRCiphertext SXREvaluator::square(SXRCiphertext sxrct) {
	seal::Ciphertext result_ct;
	
	evaluator_.square(sxrct.get_ct(), result_ct);
	evaluator_.relinearize_inplace(result_ct, relin_keys_);
	evaluator_.rescale_to_next_inplace(result_ct);
	
	SXRCiphertext result(result_ct);
	result.set_depth(sxrct.get_depth() + 1);
	return result;
}

int SXREvaluator::match_depth(SXRCiphertext& sxrctA, SXRCiphertext& sxrctB) {
	int minDepth = std::min(sxrctA.get_depth(), sxrctB.get_depth());
	int maxDepth = std::max(sxrctA.get_depth(), sxrctB.get_depth());
	
	seal::Ciphertext newCiphertext = (sxrctA.get_depth() == minDepth) ? sxrctA.get_ct() : sxrctB.get_ct();
	
	int depthDiff = maxDepth - minDepth;
	
	if (depthDiff != 0) {
		float oneFloat = 1.0f;
		
		seal::Plaintext onePlaintext;
		encoder_.encode(oneFloat, scale_, onePlaintext);
		
		seal::Ciphertext oneCiphertext;
		encryptor_.encrypt(onePlaintext, oneCiphertext);
		
		
		for (int i = 0; i < maxDepth; i++) {
			if (i >= minDepth) {
				evaluator_.multiply_inplace(newCiphertext, oneCiphertext);
				evaluator_.relinearize_inplace(newCiphertext, relin_keys_);
				evaluator_.rescale_to_next_inplace(newCiphertext);
			}
			
			evaluator_.square_inplace(oneCiphertext);
			evaluator_.relinearize_inplace(oneCiphertext, relin_keys_);
			evaluator_.rescale_to_next_inplace(oneCiphertext);
		}
	}
	
	if (sxrctA.get_depth() == minDepth) {
		sxrctA.set_ct(newCiphertext);
		sxrctA.set_depth(maxDepth);
	}
	else {
		sxrctB.set_ct(newCiphertext);
		sxrctB.set_depth(maxDepth);
	}
	
	return maxDepth;
}
