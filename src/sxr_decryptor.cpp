#include "seal_x_ros/sxr_decryptor.hpp"

SXRDecryptor::SXRDecryptor(std::vector<uint8_t> serialized_parms,
						   const seal::SecretKey &secret_key)
	: context_(context_from_parms(serialized_parms)),
	  decryptor_(*context_, secret_key), 
	  encoder_(*context_) {
}

seal::Plaintext SXRDecryptor::decrypt(std::vector<uint8_t> serialized_ct) {
	seal::Plaintext decoded_pt;
	seal::Ciphertext encrypted_ct = deserialize_to_ct(serialized_ct, context_);
	decryptor_.decrypt(encrypted_ct, decoded_pt);
	
	return decoded_pt;
}

float SXRDecryptor::decrypt_float(std::vector<uint8_t> serialized_ct) {
	seal::Plaintext decrypted_float = decrypt(serialized_ct);
	
	std::vector<double> decoded_float;
	encoder_.decode(decrypted_float, decoded_float);
	
	return static_cast<float>(decoded_float[0]);
}

std::vector<float> SXRDecryptor::decrypt_float_array(std::vector<uint8_t> serialized_ct) {
	seal::Plaintext decrypted_double_array = decrypt(serialized_ct);
	
	std::vector<double> decoded_double_array;
	encoder_.decode(decrypted_double_array, decoded_double_array);
	
	return convert_double_array_to_float(decoded_double_array);
}

