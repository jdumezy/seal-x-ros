#ifndef SEAL_PARMS_AND_KEYS_HPP
#define SEAL_PARMS_AND_KEYS_HPP

#include "seal/seal.h"
#include "seal_ros_nodes/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class ParmsAndKeysManager {
public:
	ParmsAndKeysManager();

	double get_scale() const;
	
	seal::SecretKey get_secret_key() const;
	seal::PublicKey get_public_key() const;
	seal::RelinKeys get_relin_keys() const;
	seal::GaloisKeys get_galois_keys() const;
	
	std::vector<uint8_t> get_serialized_parms() const;
	std::vector<uint8_t> get_serialized_pk() const;
	std::vector<uint8_t> get_serialized_rlk() const;
	std::vector<uint8_t> get_serialized_galk() const;

private:
	void create_parms();
	void create_keys();

	double scale_;
	std::shared_ptr<seal::SEALContext> context_;
	
	seal::SecretKey secret_key_;
	seal::PublicKey public_key_;
	seal::RelinKeys relin_keys_;
	seal::GaloisKeys galois_keys_;
	
	std::vector<uint8_t> serialized_parms_;
	std::vector<uint8_t> serialized_pk_;
	std::vector<uint8_t> serialized_rlk_;
	std::vector<uint8_t> serialized_galk_;
};

#endif // SEAL_PARMS_AND_KEYS_HPP

