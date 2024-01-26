#include "seal_ros_nodes/seal_parms_and_keys.hpp"

ParmsAndKeysManager::ParmsAndKeysManager() {
	create_parms();
	create_keys();
}

void ParmsAndKeysManager::create_parms() {
	seal::EncryptionParameters parms(seal::scheme_type::ckks);
	
	size_t poly_modulus_degree = 8192;
	parms.set_poly_modulus_degree(poly_modulus_degree);
	parms.set_coeff_modulus(seal::CoeffModulus::Create(poly_modulus_degree, { 60, 40, 40, 60 }));
	
	serialized_parms_.resize(parms.save_size());
	parms.save(reinterpret_cast<seal::seal_byte*>(serialized_parms_.data()), serialized_parms_.size());
	
	context_ = std::make_shared<seal::SEALContext>(parms);
	scale_ = std::pow(2.0, 40);
}

void ParmsAndKeysManager::create_keys() {
	seal::KeyGenerator keygen(*context_);
	secret_key_ = keygen.secret_key();
	
	keygen.create_public_key(public_key_);
	keygen.create_relin_keys(relin_keys_);
	
	serialized_pk_.resize(public_key_.save_size());
	serialized_rlk_.resize(relin_keys_.save_size());
	
	public_key_.save(reinterpret_cast<seal::seal_byte*>(serialized_pk_.data()), serialized_pk_.size());
	relin_keys_.save(reinterpret_cast<seal::seal_byte*>(serialized_rlk_.data()), serialized_rlk_.size());	
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

std::vector<uint8_t> ParmsAndKeysManager::get_serialized_parms() const {
	return serialized_parms_;
}

std::vector<uint8_t> ParmsAndKeysManager::get_serialized_pk() const {
	return serialized_pk_;
}

std::vector<uint8_t> ParmsAndKeysManager::get_serialized_rlk() const {
	return serialized_rlk_;
}

