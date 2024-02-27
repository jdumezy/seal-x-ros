// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_SEAL_X_ROS_SXR_PARMS_AND_KEYS_HPP_
#define INCLUDE_SEAL_X_ROS_SXR_PARMS_AND_KEYS_HPP_

#include <vector>
#include <memory>
#include <cstdint>

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

/**
 * @class ParmsAndKeysManager
 * @brief Manager for encryption parameters and keys in SEAL-based operations.
 *
 * Manages the creation, storage, and access of encryption parameters and keys for use in 
 * SEAL operations within a SEAL-X-ROS framework. This includes the generation of secret and 
 * public keys, relinearization keys, and Galois keys, as well as their serialization for 
 * storage or transmission. Ensures consistent encryption settings across the system.
 */
class ParmsAndKeysManager {
 public:
   /**
   * @brief Constructs a new ParmsAndKeysManager object.
   *
   * Initializes the manager by setting up the SEAL encryption environment. This includes 
   * generating encryption parameters and all necessary types of keys, and preparing their 
   * serialized versions for use in the framework.
   */
  ParmsAndKeysManager();

  /**
   * @brief Retrieves the scale used in encryption operations.
   * 
   * Returns the scale factor set during the encryption parameter generation. This is crucial 
   * for maintaining precision in encrypted computations.
   * 
   * @return The scale factor used in encoding.
   */
  double getScale() const;

  /**
   * @brief Retrieves the secret key.
   * 
   * Returns the secret key generated for use in decryption operations. Essential for 
   * decrypting data encrypted with the corresponding public key.
   * 
   * @return The generated secret key.
   */
  seal::SecretKey getSecretKey() const;

  /**
   * @brief Retrieves the public key.
   * 
   * Returns the public key generated for use in encryption operations. Essential for 
   * encrypting data intended for decryption with the corresponding secret key.
   * 
   * @return The generated public key.
   */
  seal::PublicKey getPublicKey() const;

  /**
   * @brief Retrieves the relinearization keys.
   * 
   * Returns the relinearization keys generated for use in operations that change the size of 
   * ciphertexts, such as multiplication. Essential for maintaining efficiency in encrypted computations.
   * 
   * @return The generated relinearization keys.
   */
  seal::RelinKeys getRelinKeys() const;

  /**
   * @brief Retrieves the Galois keys.
   * 
   * Returns the Galois keys generated for use in specific operations, such as rotations within 
   * encrypted vectors. Essential for supporting complex encrypted computations.
   * 
   * @return The generated Galois keys.
   */
  seal::GaloisKeys getGaloisKeys() const;

  /**
   * @brief Retrieves serialized encryption parameters.
   * 
   * Returns the serialization of the encryption parameters used to configure the SEAL environment. 
   * Essential for sharing or storing the encryption configuration.
   * 
   * @return A byte vector containing the serialized encryption parameters.
   */
  std::vector<uint8_t> getSerializedParms() const;

  /**
   * @brief Retrieves serialized public key.
   * 
   * Returns the serialization of the public key. Useful for sharing the public key across 
   * different components or systems.
   * 
   * @return A byte vector containing the serialized public key.
   */
  std::vector<uint8_t> getSerializedPk() const;

  /**
   * @brief Retrieves serialized relinearization keys.
   * 
   * Returns the serialization of the relinearization keys. Useful for sharing the relinearization keys 
   * for encrypted computations that require them.
   * 
   * @return A byte vector containing the serialized relinearization keys.
   */
  std::vector<uint8_t> getSerializedRlk() const;

  /**
   * @brief Retrieves serialized Galois keys.
   * 
   * Returns the serialization of the Galois keys. Useful for sharing the Galois keys for encrypted 
   * computations that require rotations.
   * 
   * @return A byte vector containing the serialized Galois keys.
   */
  std::vector<uint8_t> getSerializedGalk() const;

  /**
   * @brief Get len of longest possible vector.
   * 
   * Returns the length of the longest possible vector. Needed to split a message accordingly. 
   * 
   * @return size_t maximum number of elements of a vector.
   */
  size_t getMaxLen() const;

 private:
  /**
   * @brief Creates the encryption parameters.
   * 
   * Generates the encryption parameters based on pre-defined or dynamic criteria, setting up 
   * the SEAL environment for encryption operations.
   */
  void createParms();

  /**
   * @brief Creates all required keys.
   * 
   * Generates the secret key, public key, relinearization keys, and Galois keys required for 
   * SEAL operations, ensuring they are ready for encryption and decryption tasks.
   */
  void createKeys();

  size_t m_poly_modulus_degree;  ///< Poly moudulus degree
  double mScale;  ///< Scale factor for encoding.
  std::shared_ptr<seal::SEALContext> mpContext;  ///< Shared pointer to the SEAL context.

  seal::SecretKey mSecretKey;  ///< Secret key for decryption.
  seal::PublicKey mPublicKey;  ///< Public key for encryption.
  seal::RelinKeys mRelinKeys;  ///< Relinearization keys for size reduction after multiplication.
  seal::GaloisKeys mGaloisKeys;  ///< Galois keys for vector rotations.

  std::vector<uint8_t> mSerializedParms;  ///< Serialized encryption parameters.
  std::vector<uint8_t> mSerializedPk;  ///< Serialized public key.
  std::vector<uint8_t> mSerializedRlk;  ///< Serialized relinearization keys.
  std::vector<uint8_t> mSerializedGalk;  ///< Serialized Galois keys.
};

#endif  // INCLUDE_SEAL_X_ROS_SXR_PARMS_AND_KEYS_HPP_

