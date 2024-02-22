// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_SEAL_X_ROS_SXR_LIB_HPP_
#define INCLUDE_SEAL_X_ROS_SXR_LIB_HPP_

#include <vector>
#include <memory>
#include <cstdint>
#include <stdexcept>

#include "seal/seal.h"

/**
 * @brief Calculates the scale based on the encryption depth and prime factors.
 * 
 * This function calculates the appropriate scale factor to use in encryption parameters 
 * based on the depth of the computation and a vector of prime numbers. The scale factor 
 * influences the precision of encrypted computations.
 * 
 * @param depth The maximum depth of the arithmetic circuit.
 * @param scale The initial scale factor.
 * @param primeArray A vector of prime numbers used in the modulus chain.
 * @return The calculated scale factor.
 */
double calculateScale(int depth, double scale, std::vector<double> primeArray);

/**
 * @brief Calculates the depth of encryption based on the ciphertext scale and prime factors.
 * 
 * This function calculates the depth of encryption that can be supported given a ciphertext's 
 * scale and a vector of prime numbers. The depth is indicative of the complexity of computations 
 * that can be performed on the encrypted data.
 * 
 * @param ciphertextScale The scale of the ciphertext.
 * @param scale The initial scale factor.
 * @param primeArray A vector of prime numbers used in the modulus chain.
 * @return The calculated depth of encryption.
 */
int calculateDepth(double ciphertextScale, double scale, std::vector<double> primeArray);

/**
 * @brief Converts a byte array to a float array.
 * 
 * This function takes a vector of bytes and converts it into a vector of floats. This is useful 
 * for converting serialized data back into its original floating-point representation.
 * 
 * @param bytes The byte array to convert.
 * @return A vector of floats converted from the byte array.
 */
std::vector<float> byteArrayToFloatArray(const std::vector<uint8_t>& bytes);

/**
 * @brief Converts a float array to a byte array.
 * 
 * This function takes a vector of floats and serializes it into a vector of bytes. This is useful 
 * for preparing floating-point data for transmission or storage in serialized form.
 * 
 * @param floatArray The float array to convert.
 * @return A vector of bytes converted from the float array.
 */
std::vector<uint8_t> floatArrayToByteArray(const std::vector<float>& floatArray);

/**
 * @brief Converts a float array to a double array.
 * 
 * This function converts a vector of floats into a vector of doubles. This is useful when higher 
 * precision is required for subsequent calculations or operations.
 * 
 * @param floatArray The float array to convert.
 * @return A vector of doubles converted from the float array.
 */
std::vector<double> floatArrayToDoubleArray(const std::vector<float>& floatArray);

/**
 * @brief Converts a double array to a float array.
 * 
 * This function converts a vector of doubles into a vector of floats. This is useful for 
 * reducing the precision of data, typically for compatibility with certain APIs or to save space.
 * 
 * @param doubleArray The double array to convert.
 * @return A vector of floats converted from the double array.
 */
std::vector<float> doubleArrayToFloatArray(const std::vector<double>& doubleArray);

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
seal::PublicKey deserializeToPk(std::vector<uint8_t> serializedPk, seal::SEALContext* pContext);

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
seal::RelinKeys deserializeToRlk(std::vector<uint8_t> serializedRlk, seal::SEALContext* pContext);

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
seal::GaloisKeys deserializeToGalk(std::vector<uint8_t> serializedGalk, seal::SEALContext* pContext);

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
seal::Ciphertext deserializeToCt(std::vector<uint8_t> serializedCt, seal::SEALContext* pContext);

/**
 * @brief Serializes a SEAL object into a vector of bytes.
 *
 * This template function serializes an object from the SEAL library, converting it into a 
 * vector of bytes. The function is designed to work with any SEAL object that supports 
 * the `save` method, indicating its ability to be serialized. This is crucial for storing 
 * or transmitting encrypted data in a compact and efficient format.
 *
 * @tparam T The type of the SEAL object to be serialized (e.g., Ciphertext, PublicKey).
 * @param obj The SEAL object to serialize.
 * @return A vector of bytes representing the serialized form of the SEAL object.
 *
 * @note The serialized object can be deserialized using the appropriate SEAL deserialization methods.
 * Ensure that the type `T` is consistent during serialization and deserialization to avoid errors.
 */
template<typename T>
std::vector<uint8_t> serializeSealObject(const T& obj) {
  std::size_t size = obj.save_size();
  std::vector<uint8_t> serializedObject(size);
  obj.save(reinterpret_cast<seal::seal_byte*>(serializedObject.data()), size, seal::compr_mode_type::zstd);
  return serializedObject;
}

#endif  // INCLUDE_SEAL_X_ROS_SXR_LIB_HPP_

