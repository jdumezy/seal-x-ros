#include "seal_x_ros/sxr_decryptor.hpp"

SXRDecryptor::SXRDecryptor(std::vector<uint8_t> serialized_parms,
						   const seal::SecretKey &secret_key)
	: context_(context_from_parms(serialized_parms)),
	  decryptor_(*context_, secret_key), 
	  encoder_(*context_) {
}

float SXRDecryptor::decrypt_float(std::vector<uint8_t> serialized_ct) {
	seal::Plaintext decoded_pt;
	
	seal::Ciphertext encrypted_ct = deserialize_to_ct(serialized_ct, context_);
	
	decryptor_.decrypt(encrypted_ct, decoded_pt);
	
	std::vector<double> decoded_float;
	
	encoder_.decode(decoded_pt, decoded_float);
	
	return static_cast<float>(decoded_float[0]);
}

