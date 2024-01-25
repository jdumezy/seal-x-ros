#ifndef SEAL_CONTEXT_HPP
#define SEAL_CONTEXT_HPP

#include "seal/seal.h"

class ContextManager {
public:
	ContextManager();

	std::shared_ptr<seal::SEALContext> get_context() const;
	double get_scale() const;
	seal::SecretKey get_secret_key() const;
	seal::PublicKey get_public_key() const;
	seal::RelinKeys get_relin_keys() const;
	seal::GaloisKeys get_galois_keys() const;

private:
	void create_context();
	void create_keys();

	std::shared_ptr<seal::SEALContext> context_;
	double scale_;
	seal::SecretKey secret_key_;
	seal::PublicKey public_key_;
	seal::RelinKeys relin_keys_;
	seal::GaloisKeys galois_keys_;
};

#endif // SEAL_CONTEXT_HPP

