#include "seal_x_ros/sxr_encryptor.hpp"

SXREncryptor::SXREncryptor(std::vector<uint8_t> serialized_parms, 
						   std::vector<uint8_t> serialized_pk, 
						   double scale)
	: context_(context_from_parms(serialized_parms)),
	  encryptor_(*context_, deserialize_to_pk(serialized_pk, context_)), 
	  encoder_(*context_), scale_(scale) {
}

std::vector<uint8_t> SXREncryptor::encrypt_float(float input_float) {
	seal::Plaintext encoded_pt;
	encoder_.encode(input_float, scale_, encoded_pt);
	
	seal::Ciphertext encrypted_ct;
	encryptor_.encrypt(encoded_pt, encrypted_ct);
	
	std::vector<uint8_t> serialized_ct = serialize_seal_object(encrypted_ct);
	
	return serialized_ct;
}

