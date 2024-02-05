// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_evaluator.hpp"

SXREvaluator::SXREvaluator(std::vector<uint8_t> serializedParms,
                   std::vector<uint8_t> serializedPk,
                   std::vector<uint8_t> serializedRlk,
                   std::vector<uint8_t> serializedGalk,
                   double scale)
  : mpContext(pContextFromParms(serializedParms)),
    mEncryptor(*mpContext, deserializeToPk(serializedPk, mpContext)),
    mEncoder(*mpContext), mEvaluator(*mpContext), mScale(scale),
    mRelinKeys(deserializeToRlk(serializedRlk, mpContext)),
    mGaloisKeys(deserializeToGalk(serializedGalk, mpContext)) {
}

// TODO(jdumezy) add init

SXRCiphertext SXREvaluator::add(SXRCiphertext sxrctA, SXRCiphertext sxrctB) {
  int depth = matchDepth(sxrctA, sxrctB);
  seal::Ciphertext resultCiphertext;
  mEvaluator.add(sxrctA.getCiphertext(), sxrctB.getCiphertext(), resultCiphertext);
  SXRCiphertext result(resultCiphertext);
  result.setDepth(depth);
  return result;
}

SXRCiphertext SXREvaluator::multiply(SXRCiphertext sxrctA, SXRCiphertext sxrctB) {
  int depth = matchDepth(sxrctA, sxrctB);
  seal::Ciphertext resultCiphertext;
  mEvaluator.multiply(sxrctA.getCiphertext(), sxrctB.getCiphertext(), resultCiphertext);
  mEvaluator.relinearize_inplace(resultCiphertext, mRelinKeys);
  mEvaluator.rescale_to_next_inplace(resultCiphertext);
  SXRCiphertext result(resultCiphertext);
  result.setDepth(depth + 1);
  return result;
}

SXRCiphertext SXREvaluator::square(SXRCiphertext sxrct) {
  seal::Ciphertext resultCiphertext;
  mEvaluator.square(sxrct.getCiphertext(), resultCiphertext);
  mEvaluator.relinearize_inplace(resultCiphertext, mRelinKeys);
  mEvaluator.rescale_to_next_inplace(resultCiphertext);
  SXRCiphertext result(resultCiphertext);
  result.setDepth(sxrct.getDepth() + 1);
  return result;
}

int SXREvaluator::matchDepth(SXRCiphertext& sxrctA, SXRCiphertext& sxrctB) {
  int minDepth = std::min(sxrctA.getDepth(), sxrctB.getDepth());
  int maxDepth = std::max(sxrctA.getDepth(), sxrctB.getDepth());
  int depthDiff = maxDepth - minDepth;
  seal::Ciphertext newCiphertext = (sxrctA.getDepth() == minDepth) ? sxrctA.getCiphertext() : sxrctB.getCiphertext();

  if (depthDiff != 0) {
    float oneFloat = 1.0f;
    seal::Plaintext onePlaintext;
    mEncoder.encode(oneFloat, mScale, onePlaintext);
    seal::Ciphertext oneCiphertext;
    mEncryptor.encrypt(onePlaintext, oneCiphertext);
    for (int i = 0; i < maxDepth; i++) {
      if (i >= minDepth) {
        mEvaluator.multiply_inplace(newCiphertext, oneCiphertext);
        mEvaluator.relinearize_inplace(newCiphertext, mRelinKeys);
        mEvaluator.rescale_to_next_inplace(newCiphertext);
      }
      mEvaluator.square_inplace(oneCiphertext);
      mEvaluator.relinearize_inplace(oneCiphertext, mRelinKeys);
      mEvaluator.rescale_to_next_inplace(oneCiphertext);
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

