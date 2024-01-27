#include "seal_x_ros/seal_encryptor.hpp"

EncryptorManager::EncryptorManager(std::vector<uint8_t> serialized_parms, 
								   std::vector<uint8_t> serialized_pk, 
								   double scale)
	: context_(CreateSEALContextFromParameters(serialized_parms)),
	  encryptor_(*context_, deserialize_to_pk(serialized_pk, context_)), 
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

