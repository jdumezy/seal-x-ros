#ifndef SEAL_ENCRYPTOR_HPP
#define SEAL_ENCRYPTOR_HPP

#include "seal/seal.h"

class Encryptor {
public:
    Encryptor(const std::shared_ptr<seal::SEALContext>& context, const seal::PublicKey& public_key);
    seal::Ciphertext encrypt(const seal::Plaintext& plaintext) const;

private:
    std::shared_ptr<seal::SEALContext> context_;
    seal::PublicKey public_key_;
    seal::Encryptor encryptor_;
};

#endif // SEAL_ENCRYPTOR_HPP

