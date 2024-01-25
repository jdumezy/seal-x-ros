#ifndef SEAL_ENCRYPTOR_HPP
#define SEAL_ENCRYPTOR_HPP

#include "seal/seal.h"

class EncryptorManager {
public:
	EncryptorManager(std::shared_ptr<seal::SEALContext> context, 
					 const seal::PublicKey &public_key, 
					 double scale);

	seal::Ciphertext encrypt_float(float inputFloat);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Encryptor encryptor_;
	seal::CKKSEncoder encoder_;
	double scale_;
};

#endif // SEAL_ENCRYPTOR_HPP

