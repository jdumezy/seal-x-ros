#ifndef SXR_LIB_HPP
#define SXR_LIB_HPP

#include "seal/seal.h"

#include <vector>
#include <memory>
#include <cstdint>

std::shared_ptr<seal::SEALContext> CreateSEALContextFromParameters(const std::vector<uint8_t>& serialized_parms);

seal::EncryptionParameters deserialize_to_parms(std::vector<uint8_t> serialized_parms);
seal::PublicKey deserialize_to_pk(std::vector<uint8_t> serialized_pk, std::shared_ptr<seal::SEALContext> context);
seal::RelinKeys deserialize_to_rlk(std::vector<uint8_t> serialized_rlk, std::shared_ptr<seal::SEALContext> context);
seal::GaloisKeys deserialize_to_galk(std::vector<uint8_t> serialized_galk, std::shared_ptr<seal::SEALContext> context);
seal::Ciphertext deserialize_to_ct(std::vector<uint8_t> serialized_ct, std::shared_ptr<seal::SEALContext> context);

template<typename T>
std::vector<uint8_t> serialize_seal_object(const T& obj) {
	std::size_t size = obj.save_size();
	std::vector<uint8_t> serializedObject(size);
	obj.save(reinterpret_cast<seal::seal_byte*>(serializedObject.data()), size);
	return serializedObject;
}

#endif // SXR_LIB_HPP

