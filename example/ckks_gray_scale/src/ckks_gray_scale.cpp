// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "ckks_gray_scale/ckks_gray_scale.hpp"

// add input image to float vector and opposite operation


SXRCiphertext gray_scale(SXRCiphertext ciphertext, SXREncryptor encryptor,
                         SXREvaluator evaluator) {
  SXRCiphertext ciphertextOne = encryptor.rotateVector(ciphertext, 1);
  SXRCiphertext ciphertextTwo = encryptor.rotateVector(ciphertext, 2);

  SXRCiphertext firstSum = encryptor.add(ciphertext, ciphertextOne);
  SXRCiphertext secondSum = encryptor.add(firstSum, ciphertextTwo);

  SXRCiphertext scalar = encryptor.encryptFloat(0.333);
  SXRCiphertext result  = encryptor.multiply(secondSum, scalar);

  return result;
}

