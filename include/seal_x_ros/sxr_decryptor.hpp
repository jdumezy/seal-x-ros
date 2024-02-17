#ifndef SXR_DECRYPTOR_HPP_
#define SXR_DECRYPTOR_HPP_

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

/**
 * @class SXRDecryptor
 * 
 * @brief A class used to handle decryption operations with serialized objects.
 * 
 * This class uses SEAL to decrypt serialized ciphertexts into floats. 
 * It is initialized with pointers to SEAL objects.
 * 
 * @see sxr_lib for the serialization and deserialization process.
 */
class SXRDecryptor {
public:
  /**
   * @brief Constructs a new SXRDecryptor object.
   *
   * Initializes the SEAL context, decryptor, and encoder for decryption operations. This setup is crucial 
   * for converting encrypted data back into its original plaintext form.
   *
   * @param pContext Pointer to the SEALContext, encapsulating the encryption parameters.
   * @param pDecryptor Pointer to the Decryptor, used for decryption operations.
   * @param pEncoder Pointer to the CKKSEncoder, facilitating the decoding of decrypted data.
   */
  SXRDecryptor(seal::SEALContext* pContext, seal::Decryptor* pDecryptor,
               seal::CKKSEncoder* pEncoder);
  
  /**
   * @brief Default constructor for the SXRDecryptor class.
   * 
   * Creates an uninitialized SXRDecryptor object. The `init` method must be called 
   * to properly initialize the decryptor before performing any decryption operations.
   */
  SXRDecryptor();

  /**
   * @brief Initializes or reinitializes the SXRDecryptor object.
   *
   * Sets up the SEAL context, decryptor, and encoder with the necessary components for 
   * decryption operations. This method allows for the decryption environment to be set 
   * or updated after the object's construction.
   *
   * @param pContext Pointer to the SEALContext.
   * @param pDecryptor Pointer to the Decryptor.
   * @param pEncoder Pointer to the CKKSEncoder.
   */
  void init(seal::SEALContext* pContext, seal::Decryptor* pDecryptor,
            seal::CKKSEncoder* pEncoder);

  /**
   * @brief Checks if the SXRDecryptor is initialized.
   *
   * Returns a boolean indicating whether the SXRDecryptor has been properly initialized 
   * with a SEAL context, decryptor, and encoder. This check ensures that decryption operations 
   * can be safely performed.
   *
   * @return bool True if initialized, false otherwise.
   */
  bool isInit();

  /**
   * @brief Decrypts a serialized ciphertext into a floating-point number.
   *
   * Decrypts and decodes the input serialized ciphertext, returning the corresponding 
   * floating-point number. This operation is essential for retrieving the original plaintext 
   * value from encrypted data.
   *
   * @param serializedCt The serialized ciphertext to be decrypted.
   * @return float The decrypted floating-point number.
   */
  float decryptFloat(std::vector<uint8_t> serializedCt);

  /**
   * @brief Decrypts a serialized ciphertext into an array of floating-point numbers.
   *
   * Decrypts and decodes the input serialized ciphertext, returning the corresponding 
   * array of floating-point numbers. This operation allows for the decryption of vector data 
   * encrypted using the CKKS scheme.
   *
   * @param serializedCt The serialized ciphertext to be decrypted.
   * @return std::vector<float> The decrypted array of floating-point numbers.
   */
  std::vector<float> decryptFloatArray(std::vector<uint8_t> serializedCt);

private:  
  seal::SEALContext* mpContext; ///< Pointer to SEAL context.
  seal::Decryptor* mpDecryptor; ///< Pointer to SEAL decryptor.
  seal::CKKSEncoder* mpEncoder; ///< Pointer to CKKS encoder.

  /**
   * @brief Helper method to perform decryption.
   *
   * Decrypts the serialized ciphertext and returns the resulting plaintext object. 
   * This method is used internally by decryptFloat and decryptFloatArray to abstract 
   * the decryption process.
   *
   * @param serializedCt The serialized ciphertext to be decrypted.
   * @return seal::Plaintext The resulting plaintext object.
   */
  seal::Plaintext decrypt(std::vector<uint8_t> serializedCt);
};

#endif // SXR_DECRYPTOR_HPP_
//
