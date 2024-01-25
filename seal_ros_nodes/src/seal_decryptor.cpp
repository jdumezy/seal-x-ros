#include "seal_ros_nodes/seal_decryptor.hpp"

DecryptorManager::DecryptorManager(std::shared_ptr<seal::SEALContext> context, 
								   const seal::SecretKey &secret_key)
	: context_(std::move(context)),
	  decryptor_(*context_, secret_key), 
	  encoder_(*context_) {
}

float DecryptorManager::decrypt_float(seal::Ciphertext encryptedCiphertext) {
	seal::Plaintext decodedPlaintext;
	decryptor_.decrypt(encryptedCiphertext, decodedPlaintext);
	
	vector<double> decodedFloat;
	
	encoder_.decode(decodedPlaintext, decodedFloat);
	
	return static_cast<float>(decodedFloat[0]);
}

