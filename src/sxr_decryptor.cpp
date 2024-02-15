// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_decryptor.hpp"

SXRDecryptor::SXRDecryptor(seal::SEALContext* pContext, seal::Decryptor* pDecryptor,
                           seal::CKKSEncoder* pEncoder) {
  init(pContext, pDecryptor, pEncoder);
}

SXRDecryptor::SXRDecryptor() {
  mpContext = NULL;
  mpDecryptor = NULL;
  mpEncoder = NULL;
}

void SXRDecryptor::init(seal::SEALContext* pContext, seal::Decryptor* pDecryptor,
                        seal::CKKSEncoder* pEncoder) {
  mpContext = pContext;
  mpDecryptor = pDecryptor;
  mpEncoder = pEncoder;
}

bool SXRDecryptor::isInit() {
  return (mpDecryptor != NULL);
}

seal::Plaintext SXRDecryptor::decrypt(std::vector<uint8_t> serializedCt) {
  seal::Plaintext decodedPlantext;
  seal::Ciphertext encryptedCt = deserializeToCt(serializedCt, mpContext);
  mpDecryptor->decrypt(encryptedCt, decodedPlantext);
  return decodedPlantext;
}

float SXRDecryptor::decryptFloat(std::vector<uint8_t> serializedCt) {
  seal::Plaintext decryptedFloat = decrypt(serializedCt);
  std::vector<double> decodedFloat;
  mpEncoder->decode(decryptedFloat, decodedFloat);
  return static_cast<float>(decodedFloat[0]);
}

std::vector<float> SXRDecryptor::decryptFloatArray(std::vector<uint8_t> serializedCt) {
  seal::Plaintext decryptedDoubleArray = decrypt(serializedCt);
  std::vector<double> decodedDoubleArray;
  mpEncoder->decode(decryptedDoubleArray, decodedDoubleArray);
  return doubleArrayToFloatArray(decodedDoubleArray);
}

