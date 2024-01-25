#ifndef SEAL_DECRYPTOR_HPP
#define SEAL_DECRYPTOR_HPP

#include "seal/seal.h"
#include <vector>

class DecryptorManager {
public:
	DecryptorManager(std::shared_ptr<seal::SEALContext> context, 
					 const seal::SecretKey &secret_key);

	float decrypt_float(seal::Ciphertext encryptedCiphertext);

private:
	std::shared_ptr<seal::SEALContext> context_;
	seal::Decryptor decryptor_;
	seal::CKKSEncoder encoder_;
};

#endif // SEAL_DECRYPTOR_HPP

