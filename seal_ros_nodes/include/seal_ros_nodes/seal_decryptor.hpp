#ifndef SEAL_DECRYPTOR_HPP
#define SEAL_DECRYPTOR_HPP

#include "seal/seal.h"
#include <memory>

class Decryptor {
public:
    Decryptor(const std::shared_ptr<seal::SEALContext>& context, const seal::SecretKey& secret_key);
    seal::Plaintext decrypt(const seal::Ciphertext& ciphertext) const;

private:
    std::shared_ptr<seal::SEALContext> context_;
    seal::SecretKey secret_key_;
    seal::Decryptor decryptor_;
};

#endif // SEAL_DECRYPTOR_HPP

