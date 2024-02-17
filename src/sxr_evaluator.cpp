// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_evaluator.hpp"

SXREvaluator::SXREvaluator(seal::CKKSEncoder* pEncoder, seal::Encryptor* pEncryptor,
                           seal::Evaluator* pEvaluator, seal::RelinKeys* pRelinKeys,
                           seal::GaloisKeys* pGaloisKeys, double scale) {
  init(pEncoder, pEncryptor, pEvaluator, pRelinKeys, pGaloisKeys, scale);
}

SXREvaluator::SXREvaluator() {
  mpEncryptor = NULL;
  mpEncoder = NULL;
  mpEvaluator = NULL;
  mpRelinKeys = NULL;
  mpGaloisKeys = NULL;
  mScale = 0.0;
}

void SXREvaluator::init(seal::CKKSEncoder* pEncoder, seal::Encryptor* pEncryptor,
                        seal::Evaluator* pEvaluator, seal::RelinKeys* pRelinKeys,
                        seal::GaloisKeys* pGaloisKeys, double scale) {
  mpEncoder = pEncoder;
  mpEncryptor = pEncryptor;
  mpEvaluator = pEvaluator;
  mpRelinKeys = pRelinKeys;
  mpGaloisKeys = pGaloisKeys;
  mScale = scale;
}

bool SXREvaluator::isInit() {
  return (mpEvaluator != NULL);
}

SXRCiphertext SXREvaluator::add(SXRCiphertext sxrctA, SXRCiphertext sxrctB) {
  int depth = matchDepth(sxrctA, sxrctB);
  seal::Ciphertext resultCiphertext;
  mpEvaluator->add(sxrctA.getCiphertext(), sxrctB.getCiphertext(), resultCiphertext);
  SXRCiphertext result(resultCiphertext);
  result.setDepth(depth);
  return result;
}

SXRCiphertext SXREvaluator::multiply(SXRCiphertext sxrctA, SXRCiphertext sxrctB) {
  int depth = matchDepth(sxrctA, sxrctB);
  seal::Ciphertext resultCiphertext;
  mpEvaluator->multiply(sxrctA.getCiphertext(), sxrctB.getCiphertext(), resultCiphertext);
  mpEvaluator->relinearize_inplace(resultCiphertext, *mpRelinKeys);
  mpEvaluator->rescale_to_next_inplace(resultCiphertext);
  SXRCiphertext result(resultCiphertext);
  result.setDepth(depth + 1);
  return result;
}

SXRCiphertext SXREvaluator::square(SXRCiphertext sxrct) {
  seal::Ciphertext resultCiphertext;
  mpEvaluator->square(sxrct.getCiphertext(), resultCiphertext);
  mpEvaluator->relinearize_inplace(resultCiphertext, *mpRelinKeys);
  mpEvaluator->rescale_to_next_inplace(resultCiphertext);
  SXRCiphertext result(resultCiphertext);
  result.setDepth(sxrct.getDepth() + 1);
  return result;
}

SXRCiphertext SXREvaluator::rotateVector(SXRCiphertext sxrct, int steps) {
  seal::Ciphertext resultCiphertext;
  mpEvaluator->rotate_vector(sxrct.getCiphertext(), steps, *mpGaloisKeys, resultCiphertext);
  SXRCiphertext result(resultCiphertext);
  result.setDepth(sxrct.getDepth());
  return(result);
}

int SXREvaluator::matchDepth(SXRCiphertext& sxrctA, SXRCiphertext& sxrctB) {
  int minDepth = std::min(sxrctA.getDepth(), sxrctB.getDepth());
  int maxDepth = std::max(sxrctA.getDepth(), sxrctB.getDepth());
  int depthDiff = maxDepth - minDepth;
  seal::Ciphertext newCiphertext = (sxrctA.getDepth() == minDepth) ? sxrctA.getCiphertext() : sxrctB.getCiphertext();

  if (depthDiff != 0) {
    float oneFloat = 1.0f;
    seal::Plaintext onePlaintext;
    mpEncoder->encode(oneFloat, mScale, onePlaintext);
    seal::Ciphertext oneCiphertext;
    mpEncryptor->encrypt(onePlaintext, oneCiphertext);
    for (int i = 0; i < maxDepth; i++) {
      if (i >= minDepth) {
        mpEvaluator->multiply_inplace(newCiphertext, oneCiphertext);
        mpEvaluator->relinearize_inplace(newCiphertext, *mpRelinKeys);
        mpEvaluator->rescale_to_next_inplace(newCiphertext);
      }
      mpEvaluator->square_inplace(oneCiphertext);
      mpEvaluator->relinearize_inplace(oneCiphertext, *mpRelinKeys);
      mpEvaluator->rescale_to_next_inplace(oneCiphertext);
    }
  }

  if (sxrctA.getDepth() == minDepth) {
    sxrctA.setCiphertext(newCiphertext);
    sxrctA.setDepth(maxDepth);
  } else {
    sxrctB.setCiphertext(newCiphertext);
    sxrctB.setDepth(maxDepth);
  }

  return maxDepth;
}

