#ifndef SXR_ENCRYPTOR_HPP_
#define SXR_ENCRYPTOR_HPP_

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

/**
 * @class SXREncryptor
 * 
 * @brief A class used to handle encryption operations with serialized objects.
 * 
 * This class uses SEAL to encrypt floats into serialized ciphertext.
 * It is initialized with pointers to SEAL objects.
 * 
 * @see sxr_lib for the serialization and deserialization process.
 */
class SXREncryptor {
public:
  /**
   * @brief Constructs a new SXREncryptor object.
   *
   * Initializes the SEAL context, encryptor, and encoder based on provided SEAL objects. 
   * This setup is critical for encrypting data with the appropriate parameters and public key,
   * ensuring that the encryption process aligns with the specified cryptographic scheme.
   *
   * @param pEncoder Pointer to the CKKSEncoder, used for encoding floats before encryption.
   * @param pEncryptor Pointer to the Encryptor, used for the encryption process.
   * @param scale The scale factor to be used in encoding floating-point numbers, affecting precision.
   */
  SXREncryptor(seal::CKKSEncoder* pEncoder,
               seal::Encryptor* pEncryptor,
               double scale);

  /**
   * @brief Default constructor for the SXREncryptor class.
   * 
   * Creates an uninitialized SXREncryptor object. The `init` method must be called 
   * to properly initialize the encryptor before performing any encryption operations.
   */
  SXREncryptor();

  /**
   * @brief Initializes or reinitializes the SXREncryptor object.
   *
   * Sets up the encoder, encryptor, and scale factor with the necessary components for 
   * encryption operations. This method allows for the encryption environment to be set 
   * or updated after the object's construction.
   *
   * @param pEncoder Pointer to the CKKSEncoder.
   * @param pEncryptor Pointer to the Encryptor.
   * @param scale The scale factor for encoding.
   */
  void init(seal::CKKSEncoder* pEncoder,
            seal::Encryptor* pEncryptor,
            double scale);

  /**
   * @brief Checks if the SXREncryptor is initialized.
   *
   * Returns a boolean indicating whether the SXREncryptor has been properly initialized 
   * with an encoder, encryptor, and scale factor. This check is vital to ensure that 
   * encryption operations can be safely performed.
   *
   * @return bool True if initialized, false otherwise.
   */
  bool isInit();
  
  /**
   * @brief Encrypts a single floating-point number.
   *
   * Encodes and encrypts a single float into a ciphertext using the SEAL library, then 
   * serializes this ciphertext into a vector of bytes. This process ensures that floating-point 
   * numbers can be securely transmitted and processed in encrypted form.
   *
   * @param inputFloat The floating-point number to be encrypted.
   * @return std::vector<uint8_t> Serialized ciphertext of the encrypted number.
   */
  std::vector<uint8_t> encryptFloat(float inputFloat);

  /**
   * @brief Encrypts an array of floating-point numbers.
   *
   * Encodes and encrypts an array of floats into a ciphertext using the SEAL library, then 
   * serializes this ciphertext into a vector of bytes. This functionality is crucial for 
   * working with vector data in encrypted computations.
   *
   * @param inputFloatArray The array of floating-point numbers to be encrypted.
   * @return std::vector<uint8_t> Serialized ciphertext of the encrypted array.
   */
  std::vector<uint8_t> encryptFloatArray(const std::vector<float>& inputFloatArray);

private:
  seal::Encryptor* mpEncryptor; ///< Pointer to SEAL Encryptor.
  seal::CKKSEncoder* mpEncoder; ///< Pointer to CKKS encoder.
  double mScale; ///< Scale factor for encoding floating-point numbers.

  /**
   * @brief Helper method to perform encryption.
   *
   * Encrypts an encoded plaintext and serializes the resulting ciphertext. This method abstracts 
   * the encryption process, allowing for reuse in encrypting both single floats and arrays.
   *
   * @param encodedPlaintext The encoded plaintext to be encrypted.
   * @return std::vector<uint8_t> Serialized ciphertext of the encrypted data.
   */
  std::vector<uint8_t> encrypt(seal::Plaintext encodedPlaintext);
};

#endif // SXR_ENCRYPTOR_HPP_

