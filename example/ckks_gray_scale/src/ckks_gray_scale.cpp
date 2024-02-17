#include "ckks_gray_scale/ckks_gray_scale.hpp"

SXRCiphertext gray_scale(SXRCiphertext ciphertext, SXREncryptor encryptor, SXREvaluator evaluator) {
  SXRCiphertext ciphertextOne = encryptor.rotateVector(ciphertext, 1);
  SXRCiphertext ciphertextTwo = encryptor.rotateVector(ciphertext, 2);
  
  SXRCiphertext firstSum = encryptor.add(ciphertext, ciphertextOne);
  SXRCiphertext secondSum = encryptor.add(firstSum, ciphertextTwo);

  SXRCiphertext scalar = encryptor.encryptFloat(0.333);
  SXRCiphertext result  = encryptor.multiply(secondSum, scalar);

  return result;
}

