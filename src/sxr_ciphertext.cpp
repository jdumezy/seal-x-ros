// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_ciphertext.hpp"

SXRCiphertext::SXRCiphertext(seal::Ciphertext ciphertext)
  : mCiphertext(ciphertext) {
  mDepth = 0;
}

seal::Ciphertext SXRCiphertext::getCiphertext() {
  return mCiphertext;
}

int SXRCiphertext::getDepth() {
  return mDepth;
}

void SXRCiphertext::setCiphertext(seal::Ciphertext newCiphertext) {
  mCiphertext = newCiphertext;
}

void SXRCiphertext::setDepth(int newDepth) {
  mDepth = newDepth;
}
