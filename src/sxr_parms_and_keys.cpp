// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_parms_and_keys.hpp"

ParmsAndKeysManager::ParmsAndKeysManager() {
    createParms();
    createKeys();
}

void ParmsAndKeysManager::createParms() {
    seal::EncryptionParameters parms(seal::scheme_type::ckks);
    size_t poly_modulus_degree = 16384;
    parms.set_poly_modulus_degree(poly_modulus_degree);
    parms.set_coeff_modulus(seal::CoeffModulus::Create(poly_modulus_degree, { 60, 40, 40, 40, 40, 60 }));
    mScale = std::pow(2.0, 40);

    mSerializedParms = serializeSealObject(parms);
    mpContext = std::make_shared<seal::SEALContext>(parms);
}

void ParmsAndKeysManager::createKeys() {
    seal::KeyGenerator keygen(*mpContext);
    mSecretKey = keygen.secret_key();
    keygen.create_public_key(mPublicKey);
    keygen.create_relin_keys(mRelinKeys);
    keygen.create_galois_keys(mGaloisKeys);

    mSerializedPk = serializeSealObject(mPublicKey);
    mSerializedRlk = serializeSealObject(mRelinKeys);
    mSerializedGalk = serializeSealObject(mGaloisKeys);
}

double ParmsAndKeysManager::getScale() const {
    return mScale;
}

seal::SecretKey ParmsAndKeysManager::getSecretKey() const {
    return mSecretKey;
}

seal::PublicKey ParmsAndKeysManager::getPublicKey() const {
    return mPublicKey;
}

seal::RelinKeys ParmsAndKeysManager::getRelinKeys() const {
    return mRelinKeys;
}

seal::GaloisKeys ParmsAndKeysManager::getGaloisKeys() const {
    return mGaloisKeys;
}

std::vector<uint8_t> ParmsAndKeysManager::getSerializedParms() const {
    return mSerializedParms;
}

std::vector<uint8_t> ParmsAndKeysManager::getSerializedPk() const {
    return mSerializedPk;
}

std::vector<uint8_t> ParmsAndKeysManager::getSerializedRlk() const {
    return mSerializedRlk;
}

std::vector<uint8_t> ParmsAndKeysManager::getSerializedGalk() const {
    return mSerializedGalk;
}

