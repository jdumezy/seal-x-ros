#include "seal_x_ros/sxr_parms_and_keys.hpp"

ParmsAndKeysManager::ParmsAndKeysManager() {
	create_parms();
	create_keys();
}

void ParmsAndKeysManager::create_parms() {
	seal::EncryptionParameters parms(seal::scheme_type::ckks);
	
	size_t poly_modulus_degree = 8192;
	parms.set_poly_modulus_degree(poly_modulus_degree);
	parms.set_coeff_modulus(seal::CoeffModulus::Create(poly_modulus_degree, { 60, 40, 40, 60 }));
	
	serialized_parms_ = serialize_seal_object(parms);
	
	context_ = std::make_shared<seal::SEALContext>(parms);
	scale_ = std::pow(2.0, 40);
}

void ParmsAndKeysManager::create_keys() {
	seal::KeyGenerator keygen(*context_);
	secret_key_ = keygen.secret_key();
	
	keygen.create_public_key(public_key_);
	keygen.create_relin_keys(relin_keys_);
	keygen.create_galois_keys(galois_keys_);
	
	serialized_pk_ = serialize_seal_object(public_key_);
	serialized_rlk_ = serialize_seal_object(relin_keys_);
	serialized_galk_ = serialize_seal_object(galois_keys_);
}

double ParmsAndKeysManager::get_scale() const {
	return scale_;
}

seal::SecretKey ParmsAndKeysManager::get_secret_key() const {
	return secret_key_;
}

seal::PublicKey ParmsAndKeysManager::get_public_key() const {
	return public_key_;
}

seal::RelinKeys ParmsAndKeysManager::get_relin_keys() const {
	return relin_keys_;
}

seal::GaloisKeys ParmsAndKeysManager::get_galois_keys() const {
	return galois_keys_;
}

std::vector<uint8_t> ParmsAndKeysManager::get_serialized_parms() const {
	return serialized_parms_;
}

std::vector<uint8_t> ParmsAndKeysManager::get_serialized_pk() const {
	return serialized_pk_;
}

std::vector<uint8_t> ParmsAndKeysManager::get_serialized_rlk() const {
	return serialized_rlk_;
}

std::vector<uint8_t> ParmsAndKeysManager::get_serialized_galk() const {
	return serialized_galk_;
}

