// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_SEAL_X_ROS_SXR_EVALUATOR_HPP_
#define INCLUDE_SEAL_X_ROS_SXR_EVALUATOR_HPP_

#include <vector>
#include <memory>
#include <cstdint>

#include "seal/seal.h"

#include "seal_x_ros/sxr_ciphertext.hpp"
#include "seal_x_ros/sxr_lib.hpp"

/**
 * @class SXREvaluator
 * 
 * @brief A class used for evaluating encrypted operations on serialized objects.
 * 
 * This class utilizes SEAL to perform operations on encrypted data, 
 * such as squaring, while working with SXRCiphertexts. 
 * It is initialized with pointers to SEAL objects.
 * 
 * @see sxr_lib for the serialization and deserialization process.
 * @see sxr_ciphertext for the SXRCiphertext definition
 */
class SXREvaluator {
 public:
  /**
   * @brief Constructs a new SXREvaluator object.
   *
   * Initializes the SEAL context, encryptor, evaluator, and encoder using the provided 
   * serialized parameters, public key, relinearization keys, and Galois keys. 
   * The scale factor for encoding is also set during construction.
   *
   * @param pEncoder Pointer to CKKSEncoder for encoding and decoding.
   * @param pEncryptor Pointer to Encryptor for encryption.
   * @param pEvaluator Pointer to Evaluator for performing operations on ciphertexts.
   * @param pRelinKeys Pointer to RelinKeys for relinearization.
   * @param pGaloisKeys Pointer to GaloisKeys for rotations.
   * @param scale The scale factor to be used in encoding and operations.
   */
  SXREvaluator(seal::CKKSEncoder* pEncoder,
               seal::Encryptor* pEncryptor,
               seal::Evaluator* pEvaluator,
               seal::RelinKeys* pRelinKeys,
               seal::GaloisKeys* pGaloisKeys,
               double scale);

  /**
   * @brief Default constructor for SXREvaluator.
   * 
   * Creates an uninitialized SXREvaluator object. The `init` method must be called
   * to initialize the evaluator before performing any operations.
   */
  SXREvaluator();

  /**
   * @brief Initializes an SXREvaluator object.
   *
   * This method is used to initialize or reinitialize an SXREvaluator object with 
   * new parameters. It sets up the SEAL context and related objects for encryption,
   * evaluation, and encoding.
   *
   * @param pEncoder Pointer to CKKSEncoder for encoding and decoding.
   * @param pEncryptor Pointer to Encryptor for encryption.
   * @param pEvaluator Pointer to Evaluator for performing operations on ciphertexts.
   * @param pRelinKeys Pointer to RelinKeys for relinearization.
   * @param pGaloisKeys Pointer to GaloisKeys for rotations.
   * @param scale The scale factor to be used in encoding and operations.
   */
  void init(seal::CKKSEncoder* pEncoder,
            seal::Encryptor* pEncryptor,
            seal::Evaluator* pEvaluator,
            seal::RelinKeys* pRelinKeys,
            seal::GaloisKeys* pGaloisKeys,
            double scale);

  /**
   * @brief Checks if the SXREvaluator is initialized.
   *
   * This method returns a boolean indicating whether the SXREvaluator object has been 
   * properly initialized with all necessary components to perform encrypted operations.
   *
   * @return bool True if initialized, false otherwise.
   */
  bool isInit();

  /**
   * @brief Performs encrypted addition on two SXRCiphertexts.
   *
   * Accepts two SXRCiphertexts, adds them together, and returns the result as a new 
   * SXRCiphertext.
   *
   * @param sxrctA First ciphertext for addition.
   * @param sxrctB Second ciphertext for addition.
   * @return SXRCiphertext Result of the addition.
   */
  SXRCiphertext add(SXRCiphertext sxrctA, SXRCiphertext sxrctB);

  /**
   * @brief Performs encrypted multiplication on two SXRCiphertexts.
   *
   * Accepts two SXRCiphertexts, multiplies them, and returns the result as a new 
   * SXRCiphertext. This operation may require relinearization.
   *
   * @param sxrctA First ciphertext for multiplication.
   * @param sxrctB Second ciphertext for multiplication.
   * @return SXRCiphertext Result of the multiplication.
   */
  SXRCiphertext multiply(SXRCiphertext sxrctA, SXRCiphertext sxrctB);

  /**
   * @brief Performs the squaring operation on encrypted data.
   *
   * Accepts an SXRCiphertext, performs the squaring operation on it, 
   * and then returns the result as an SXRCiphertext.
   *
   * @param sxrct Serialized ciphertext to be squared.
   * @return SXRCiphertext ciphertext of the squared result.
   */
  SXRCiphertext square(SXRCiphertext sxrct);

  /**
   * @brief Rotates the encrypted vector within an SXRCiphertext.
   *
   * Performs a rotation operation on the encrypted vector represented by an 
   * SXRCiphertext. The direction and magnitude of the rotation are determined by 
   * the steps parameter.
   *
   * @param sxrct The ciphertext containing the encrypted vector to be rotated.
   * @param steps The number of steps to rotate the vector; positive for left rotations, 
   * negative for right rotations.
   * @return SXRCiphertext The result of the rotation as a new ciphertext.
   */
  SXRCiphertext rotateVector(SXRCiphertext sxrct, int steps);

  /**
   * @brief Matches the encryption levels of two SXRCiphertexts.
   *
   * Adjusts the encryption levels of two SXRCiphertexts to match, allowing for operations 
   * to be performed on them. Returns the matched level.
   *
   * @param sxrctA Reference to the first ciphertext.
   * @param sxrctB Reference to the second ciphertext.
   * @return int The matched encryption level of the two ciphertexts.
   */
  int matchDepth(SXRCiphertext& sxrctA, SXRCiphertext& sxrctB);

 private:
  seal::CKKSEncoder* mpEncoder;  ///< Pointer to CKKS encoder
  seal::Encryptor* mpEncryptor;  ///< Pointer to SEAL encryptor
  seal::Evaluator* mpEvaluator;  ///< Pointer to SEAL evaluator
  seal::RelinKeys* mpRelinKeys;  ///< Pointer to relinearization keys
  seal::GaloisKeys* mpGaloisKeys;  ///< Pointer to Galois keys
  double mScale;  ///< Scale factor for CKKKS encryption scheme
};

#endif  // INCLUDE_SEAL_X_ROS_SXR_EVALUATOR_HPP_

