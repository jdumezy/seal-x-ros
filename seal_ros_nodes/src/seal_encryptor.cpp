#include "seal_ros_nodes/seal_encryptor.hpp"

EncryptorManager::EncryptorManager(std::vector<uint8_t> serialized_parms, 
								   const seal::PublicKey &public_key, 
								   double scale)
	: context_(CreateSEALContextFromParameters(serialized_parms)),
	  encryptor_(*context_, public_key), 
	  encoder_(*context_), scale_(scale) {
}

seal::Ciphertext EncryptorManager::encrypt_float(float inputFloat) {
	seal::Plaintext encodedPlaintext;
	encoder_.encode(inputFloat, scale_, encodedPlaintext);
	
	seal::Ciphertext encryptedCiphertext;
	encryptor_.encrypt(encodedPlaintext, encryptedCiphertext);
	return encryptedCiphertext;
}

