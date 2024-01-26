#include "seal_ros_nodes/seal_encryptor.hpp"

EncryptorManager::EncryptorManager(std::vector<uint8_t> serialized_parms, 
								   const seal::PublicKey &public_key, 
								   double scale)
	: context_(CreateSEALContextFromParameters(serialized_parms)),
	  encryptor_(*context_, public_key), 
	  encoder_(*context_), scale_(scale) {
}

std::vector<uint8_t> EncryptorManager::encrypt_float(float inputFloat) {
	seal::Plaintext encodedPlaintext;
	encoder_.encode(inputFloat, scale_, encodedPlaintext);
	
	seal::Ciphertext encryptedCiphertext;
	encryptor_.encrypt(encodedPlaintext, encryptedCiphertext);
	
	std::vector<uint8_t> serializedCiphertext = serialize_seal_object(encryptedCiphertext);
	
	return serializedCiphertext;
}

