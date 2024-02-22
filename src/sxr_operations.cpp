// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_operations.hpp"

SXRCiphertext gray_scale(SXRCiphertext ciphertext,
                         SXREvaluator* pEvaluator) {
  SXRCiphertext ciphertextOne = pEvaluator->rotateVector(ciphertext, 1);
  SXRCiphertext ciphertextTwo = pEvaluator->rotateVector(ciphertext, 2);

  SXRCiphertext firstSum = pEvaluator->add(ciphertext, ciphertextOne);
  SXRCiphertext secondSum = pEvaluator->add(firstSum, ciphertextTwo);

  SXRCiphertext result  = pEvaluator->multiplyFloat(secondSum, 0.3333333f);

  return result;
}

