#ifndef SXR_EVALUATOR_HPP_
#define SXR_EVALUATOR_HPP_

#include "seal/seal.h"

#include "seal_x_ros/sxr_ciphertext.hpp"
#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

/**
 * @class SXREvaluator
 * 
 * @brief A class used for evaluating encrypted operations on serialized objects.
 * 
 * This class utilizes SEAL to perform operations on encrypted data, 
 * such as squaring, while working with serialized ciphertexts. 
 * It is initialized with serialized parameters, public key, 
 * relinearization keys, and Galois keys for efficient and secure operations.
 * 
 * @see sxr_lib for the serialization and deserialization process.
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
   * @param serializedParms Serialized SEAL encryption parameters.
   * @param serializedPk Serialized public key for encryption.
   * @param serializedRlk Serialized relinearization keys.
   * @param serializedGalk Serialized Galois keys.
   * @param scale The scale factor to be used in encoding and operations.
   */
  SXREvaluator(seal::CKKSEncoder* pEncoder, seal::Encryptor* pEncryptor,
               seal::Evaluator* evaluator,  seal::RelinKeys* pRelinKeys,
               seal::GaloisKeys* pGaloisKeys, double scale);
 
  SXREvaluator();

  void init(seal::CKKSEncoder* pEncoder, seal::Encryptor* pEncryptor,
            seal::Evaluator* pEvaluator, seal::RelinKeys* pRelinKeys,
            seal::GaloisKeys* pGaloisKeys, double scale);

  bool isInit();

  SXRCiphertext add(SXRCiphertext sxrctA, SXRCiphertext sxrctB);
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
  
  int matchDepth(SXRCiphertext& sxrctA, SXRCiphertext& sxrctB);

private:
  seal::CKKSEncoder* mpEncoder;
  seal::Encryptor* mpEncryptor;
  seal::Evaluator* mpEvaluator;
  seal::RelinKeys* mpRelinKeys;
  seal::GaloisKeys* mpGaloisKeys; 
  double mScale;
};

#endif // SXR_EVALUATOR_HPP_

