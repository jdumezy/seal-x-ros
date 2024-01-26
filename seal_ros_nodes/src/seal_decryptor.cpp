#include "seal_ros_nodes/seal_decryptor.hpp"

DecryptorManager::DecryptorManager(std::vector<uint8_t> serialized_parms,
								   const seal::SecretKey &secret_key)
	: context_(CreateSEALContextFromParameters(serialized_parms)),
	  decryptor_(*context_, secret_key), 
	  encoder_(*context_) {
}

float DecryptorManager::decrypt_float(seal::Ciphertext encryptedCiphertext) {
	seal::Plaintext decodedPlaintext;
	decryptor_.decrypt(encryptedCiphertext, decodedPlaintext);
	
	std::vector<double> decodedFloat;
	
	encoder_.decode(decodedPlaintext, decodedFloat);
	
	return static_cast<float>(decodedFloat[0]);
}

