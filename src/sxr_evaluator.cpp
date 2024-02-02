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

std::vector<uint8_t> SXREvaluator::add(std::vector<uint8_t> sct_a, std::vector<uint8_t> sct_b) {
	seal::Ciphertext result;
	seal::Ciphertext ciphertext_a = deserialize_to_ct(sct_a, context_);
	seal::Ciphertext ciphertext_b = deserialize_to_ct(sct_b, context_);
	
	evaluator_.add(ciphertext_a, ciphertext_b, result);
	return serialize_seal_object(result);
}

std::vector<uint8_t> SXREvaluator::multiply(std::vector<uint8_t> sct_a, std::vector<uint8_t> sct_b) {
	seal::Ciphertext result;
	seal::Ciphertext ciphertext_a = deserialize_to_ct(sct_a, context_);
	seal::Ciphertext ciphertext_b = deserialize_to_ct(sct_b, context_);
	
	evaluator_.multiply(ciphertext_a, ciphertext_b, result);
	evaluator_.relinearize_inplace(result, relin_keys_);
	// evaluator_.rescale_to_next_inplace(result);
	return serialize_seal_object(result);
}

std::vector<uint8_t> SXREvaluator::square(std::vector<uint8_t> serialized_ct) {
	seal::Ciphertext result;
	evaluator_.square(deserialize_to_ct(serialized_ct, context_), result);
	evaluator_.relinearize_inplace(result, relin_keys_);
	// evaluator_.rescale_to_next_inplace(result);
	return serialize_seal_object(result);
}

