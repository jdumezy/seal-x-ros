#ifndef SXR_LIB_HPP
#define SXR_LIB_HPP

#include "seal/seal.h"

#include <vector>
#include <memory>
#include <cstdint>
#include <chrono>
#include <sstream>
#include <functional>

/**
 * @brief Create a SEALContext from serialized parameters.
 * 
 * This function generates a shared pointer of a SEALContext from a vector of bytes representing 
 * the serialized form of a SEAL EncryptionParameters object.
 * 
 * @param serialized_parms Serialized encryption parameters.
 * @return Shared pointer of the generated SEALContext.
 */
std::shared_ptr<seal::SEALContext> context_from_parms(const std::vector<uint8_t>& serialized_parms);

/**
 * @brief Deserialization of serialized parameters.
 * 
 * This function generates a SEAL EncryptionParameters object from a vector of bytes representing 
 * the serialized form of a SEAL EncryptionParameters object.
 * 
 * @param serialized_parms Serialized encryption parameters.
 * @return Deserialized encryption parameters.
 */
seal::EncryptionParameters deserialize_to_parms(std::vector<uint8_t> serialized_parms);

/**
 * @brief Deserialization of a serialized public key.
 * 
 * This function generates a SEAL PublicKey object from a vector of bytes representing 
 * the serialized form of a SEAL PublicKey object.
 * 
 * @param serialized_pk Serialized public key.
 * @param context Shared pointer of the context.
 * @return Deserialized public key.
 */
seal::PublicKey deserialize_to_pk(std::vector<uint8_t> serialized_pk, std::shared_ptr<seal::SEALContext> context);

/**
 * @brief Deserialization of serialized relinearization keys.
 * 
 * This function generates a SEAL RelinKeys object from a vector of bytes representing 
 * the serialized form of a SEAL RelinKeys object.
 * 
 * @param serialized_rlk Serialized relinearization keys.
 * @param context Shared pointer of the context.
 * @return Deserialized relinearization keys.
 */
seal::RelinKeys deserialize_to_rlk(std::vector<uint8_t> serialized_rlk, std::shared_ptr<seal::SEALContext> context);

/**
 * @brief Deserialization of serialized galois keys.
 * 
 * This function generates a SEAL GaloisKeys object from a vector of bytes representing 
 * the serialized form of a SEAL GaloisKeys object.
 * 
 * @param serialized_galk Serialized galois keys.
 * @param context Shared pointer of the context.
 * @return Deserialized galois keys.
 */
seal::GaloisKeys deserialize_to_galk(std::vector<uint8_t> serialized_galk, std::shared_ptr<seal::SEALContext> context);

/**
 * @brief Deserialization of a serialized ciphertext.
 * 
 * This function generates a SEAL Ciphertext object from a vector of bytes representing 
 * the serialized form of a SEAL Ciphertext object.
 * 
 * @param serialized_ct Serialized ciphertext.
 * @param context Shared pointer of the context.
 * @return Deserialized ciphertext.
 */
seal::Ciphertext deserialize_to_ct(std::vector<uint8_t> serialized_ct, std::shared_ptr<seal::SEALContext> context);

/**
 * @brief Serializes a SEAL object into a vector of bytes.
 *
 * This template function serializes an object from the SEAL library, converting it into a 
 * vector of bytes. The function is designed to work with any SEAL object that supports 
 * the `save` method, `save_size` and `load` method. This serialization is useful for storing and 
 * transmitting encrypted data. Cf the serialization example in the SEAL library.
 *
 * @tparam T The type of the SEAL object to be serialized.
 * @param obj The SEAL object to serialize.
 * @return A vector of bytes representing the serialized form of the input SEAL object.
 *
 * @note The serialized object can be deserialized using the appropriate SEAL deserialization methods.
 *       Ensure that the type `T` is consistent during serialization and deserialization.
 */
template<typename T>
std::vector<uint8_t> serialize_seal_object(const T& obj) {
	std::size_t size = obj.save_size();
	std::vector<uint8_t> serialized_object(size);
	obj.save(reinterpret_cast<seal::seal_byte*>(serialized_object.data()), size);
	return serialized_object;
}

#endif // SXR_LIB_HPP

