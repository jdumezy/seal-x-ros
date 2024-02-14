#ifndef SXR_LIB_HPP_
#define SXR_LIB_HPP_

#include "seal/seal.h"

#include <vector>
#include <memory>
#include <cstdint>
#include <stdexcept>

double calculateScale(int depth, double scale, std::vector<double> primeArray);
int calculateDepth(double ciphertextScale, double scale, std::vector<double> primeArray);

std::vector<float> byteArrayToFloatArray(const std::vector<uint8_t>& bytes);
std::vector<uint8_t> floatArrayToByteArray(const std::vector<float>& floatArray);
std::vector<double> floatArrayToDoubleArray(const std::vector<float>& floatArray);
std::vector<float> doubleArrayToFloatArray(const std::vector<double>& doubleArray);

/**
 * @brief Create a SEALContext from serialized parameters.
 * 
 * This function generates a shared pointer of a SEALContext from a vector of bytes representing 
 * the serialized form of a SEAL EncryptionParameters object.
 * 
 * @param serializedParms Serialized encryption parameters.
 * @return Shared pointer of the generated SEALContext.
 */
std::shared_ptr<seal::SEALContext> pContextFromParms(const std::vector<uint8_t>& serializedParms);

/**
 * @brief Deserialization of serialized parameters.
 * 
 * This function generates a SEAL EncryptionParameters object from a vector of bytes representing 
 * the serialized form of a SEAL EncryptionParameters object.
 * 
 * @param serializedParms Serialized encryption parameters.
 * @return Deserialized encryption parameters.
 */
seal::EncryptionParameters deserializeToParms(std::vector<uint8_t> serializedParms);

/**
 * @brief Deserialization of a serialized public key.
 * 
 * This function generates a SEAL PublicKey object from a vector of bytes representing 
 * the serialized form of a SEAL PublicKey object.
 * 
 * @param serializedPk Serialized public key.
 * @param context Shared pointer of the context.
 * @return Deserialized public key.
 */
seal::PublicKey deserializeToPk(std::vector<uint8_t> serializedPk, std::shared_ptr<seal::SEALContext> context);

/**
 * @brief Deserialization of serialized relinearization keys.
 * 
 * This function generates a SEAL RelinKeys object from a vector of bytes representing 
 * the serialized form of a SEAL RelinKeys object.
 * 
 * @param serializedRlk Serialized relinearization keys.
 * @param context Shared pointer of the context.
 * @return Deserialized relinearization keys.
 */
seal::RelinKeys deserializeToRlk(std::vector<uint8_t> serializedRlk, std::shared_ptr<seal::SEALContext> context);

/**
 * @brief Deserialization of serialized galois keys.
 * 
 * This function generates a SEAL GaloisKeys object from a vector of bytes representing 
 * the serialized form of a SEAL GaloisKeys object.
 * 
 * @param serializedGalk Serialized galois keys.
 * @param context Shared pointer of the context.
 * @return Deserialized galois keys.
 */
seal::GaloisKeys deserializeToGalk(std::vector<uint8_t> serializedGalk, std::shared_ptr<seal::SEALContext> context);

/**
 * @brief Deserialization of a serialized ciphertext.
 * 
 * This function generates a SEAL Ciphertext object from a vector of bytes representing 
 * the serialized form of a SEAL Ciphertext object.
 * 
 * @param serializedCt Serialized ciphertext.
 * @param context Shared pointer of the context.
 * @return Deserialized ciphertext.
 */
seal::Ciphertext deserializeToCt(std::vector<uint8_t> serializedCt, std::shared_ptr<seal::SEALContext> context);

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
 *     Ensure that the type `T` is consistent during serialization and deserialization.
 */
template<typename T>
std::vector<uint8_t> serializeSealObject(const T& obj) {
  std::size_t size = obj.save_size();
  std::vector<uint8_t> serializedObject(size);
  obj.save(reinterpret_cast<seal::seal_byte*>(serializedObject.data()), size, seal::compr_mode_type::zstd);
  return serializedObject;
}

#endif // SXR_LIB_HPP_

