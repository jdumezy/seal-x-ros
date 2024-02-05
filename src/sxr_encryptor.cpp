// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_encryptor.hpp"

SXREncryptor::SXREncryptor(std::vector<uint8_t> serializedParms,
               std::vector<uint8_t> serializedPk,
               double scale)
  : mpContext(pContextFromParms(serializedParms)),
    mEncryptor(*mpContext, deserializeToPk(serializedPk, mpContext)),
    mEncoder(*mpContext), mScale(scale) {
}

std::vector<uint8_t> SXREncryptor::encrypt(seal::Plaintext encodedPlaintext) {
  seal::Ciphertext encryptedCt;
  mEncryptor.encrypt(encodedPlaintext, encryptedCt);
  std::vector<uint8_t> serializedCt = serializeSealObject(encryptedCt);
  return serializedCt;
}

std::vector<uint8_t> SXREncryptor::encryptFloat(float inputFloat) {
  seal::Plaintext encodedPlaintext;
  mEncoder.encode(inputFloat, mScale, encodedPlaintext);
  return encrypt(encodedPlaintext);
}

std::vector<uint8_t> SXREncryptor::encryptFloatArray(const std::vector<float>& inputFloatArray) {
  std::vector<double> doubleArray = floatArrayToDoubleArray(inputFloatArray);
  size_t slotCount = mEncoder.slot_count();
  if (doubleArray.size() > slotCount / 2) {
    throw std::runtime_error("Input array is too large for CKKS slots");
  }
  seal::Plaintext encodedPlaintext;
  mEncoder.encode(doubleArray, mScale, encodedPlaintext);
  return encrypt(encodedPlaintext);
}

