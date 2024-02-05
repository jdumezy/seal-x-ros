// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_decryptor.hpp"

SXRDecryptor::SXRDecryptor(std::vector<uint8_t> serializedParms,
                           const seal::SecretKey &secretKey)
    : mpContext(pContextFromParms(serializedParms)),
      mDecryptor(*mpContext, secretKey),
      mEncoder(*mpContext) {
}

seal::Plaintext SXRDecryptor::decrypt(std::vector<uint8_t> serializedCt) {
    seal::Plaintext decodedPlantext;
    seal::Ciphertext encryptedCt = deserializeToCt(serializedCt, mpContext);
    mDecryptor.decrypt(encryptedCt, decodedPlantext);
    return decodedPlantext;
}

float SXRDecryptor::decryptFloat(std::vector<uint8_t> serializedCt) {
    seal::Plaintext decryptedFloat = decrypt(serializedCt);
    std::vector<double> decodedFloat;
    mEncoder.decode(decryptedFloat, decodedFloat);
    return static_cast<float>(decodedFloat[0]);
}

std::vector<float> SXRDecryptor::decryptFloatArray(std::vector<uint8_t> serializedCt) {
    seal::Plaintext decryptedDoubleArray = decrypt(serializedCt);
    std::vector<double> decodedDoubleArray;
    mEncoder.decode(decryptedDoubleArray, decodedDoubleArray);
    return doubleArrayToFloatArray(decodedDoubleArray);
}

