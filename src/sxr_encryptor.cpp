// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_encryptor.hpp"

SXREncryptor::SXREncryptor(seal::CKKSEncoder* pEncoder,
                           seal::Encryptor* pEncryptor,
                           double scale) {
  init(pEncoder, pEncryptor, scale);
}

SXREncryptor::SXREncryptor() {
  mpEncoder = NULL;
  mpEncryptor = NULL;
  mScale = 0.0;
}

void SXREncryptor::init(seal::CKKSEncoder* pEncoder,
                        seal::Encryptor* pEncryptor,
                        double scale) {
  mpEncoder = pEncoder;
  mpEncryptor = pEncryptor;
  mScale = scale;
}

bool SXREncryptor::isInit() {
  return (mpEncoder != NULL);
}

std::vector<uint8_t> SXREncryptor::encrypt(seal::Plaintext encodedPlaintext) {
  seal::Ciphertext encryptedCt;
  mpEncryptor->encrypt(encodedPlaintext, encryptedCt);
  std::vector<uint8_t> serializedCt = serializeSealObject(encryptedCt);
  return serializedCt;
}

std::vector<uint8_t> SXREncryptor::encryptFloat(float inputFloat) {
  seal::Plaintext encodedPlaintext;
  mpEncoder->encode(inputFloat, mScale, encodedPlaintext);
  return encrypt(encodedPlaintext);
}

std::vector<uint8_t> SXREncryptor::encryptFloatArray(const std::vector<float>& inputFloatArray) {
  std::vector<double> doubleArray = floatArrayToDoubleArray(inputFloatArray);
  size_t slotCount = mpEncoder->slot_count();
  if (doubleArray.size() > slotCount) {
    throw std::runtime_error("Input array is too large for CKKS slots");
  }
  seal::Plaintext encodedPlaintext;
  mpEncoder->encode(doubleArray, mScale, encodedPlaintext);
  return encrypt(encodedPlaintext);
}

