#include "seal_x_ros/sxr_evaluator.hpp"

SXREvaluator::SXREvaluator(std::vector<uint8_t> serialized_parms, 
								   std::vector<uint8_t> serialized_pk,
								   std::vector<uint8_t> serialized_rlk,
								   std::vector<uint8_t> serialized_galk, 
								   double scale)
	: context_(context_from_parms(serialized_parms)),
	  encryptor_(*context_, deserialize_to_pk(serialized_pk, context_)), 
	  encoder_(*context_), evaluator_(*context_), scale_(scale) {
	
	relin_keys_ = deserialize_to_rlk(serialized_rlk, context_);
	galois_keys_ = deserialize_to_galk(serialized_galk, context_);
}

std::vector<uint8_t> SXREvaluator::square(std::vector<uint8_t> serialized_ct) {
	seal::Ciphertext result;
	evaluator_.square(deserialize_to_ct(serialized_ct, context_), result);
	evaluator_.relinearize_inplace(result, relin_keys_);
	return serialize_seal_object(result);
}

