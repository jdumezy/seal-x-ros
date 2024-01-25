#include "seal_ros_nodes/seal_context.hpp"

ContextManager::ContextManager() {
	create_context();
	create_keys();
}

void ContextManager::create_context() {
	seal::EncryptionParameters parms(seal::scheme_type::ckks);
	
	size_t poly_modulus_degree = 8192;
	parms.set_poly_modulus_degree(poly_modulus_degree);
	parms.set_coeff_modulus(seal::CoeffModulus::Create(poly_modulus_degree, { 60, 40, 40, 60 }));
	
	scale_ = std::pow(2.0, 40);
	
	context_ = std::make_shared<seal::SEALContext>(parms);
}

void ContextManager::create_keys() {
	seal::KeyGenerator keygen(*context_);
	secret_key_ = keygen.secret_key();
	
	seal::PublicKey public_key;
	keygen.create_public_key(public_key_);
	
	seal::RelinKeys relin_keys;
	keygen.create_relin_keys(relin_keys_);
	
	seal::GaloisKeys galois_keys;
	keygen.create_galois_keys(galois_keys_);
}

std::shared_ptr<seal::SEALContext> ContextManager::get_context() const {
	return context_;
}

double ContextManager::get_scale() const {
	return scale_;
}

seal::SecretKey ContextManager::get_secret_key() const {
	return secret_key_;
}

seal::PublicKey ContextManager::get_public_key() const {
	return public_key_;
}

seal::RelinKeys ContextManager::get_relin_keys() const {
	return relin_keys_;
}

seal::GaloisKeys ContextManager::get_galois_keys() const {
	return galois_keys_;
}

