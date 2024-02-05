#ifndef SXR_PARMS_AND_KEYS_HPP_
#define SXR_PARMS_AND_KEYS_HPP_

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

/**
 * @class ParmsAndKeysManager
 * @brief Manager for encryption parameters and keys in SEAL-based operations.
 *
 * This class is responsible for generating, storing, and providing access to encryption
 * parameters and keys used in SEAL operations.
 * It ensures that all components in the system use consistent and properly 
 * configured encryption settings.
 */
class ParmsAndKeysManager {
public:
     /**
     * @brief Constructs a new ParmsAndKeysManager object.
     *
     * Sets up the SEAL environment and generates the necessary encryption parameters and keys.
     */
    ParmsAndKeysManager();

    double getScale() const;
    
    seal::SecretKey getSecretKey() const;
    seal::PublicKey getPublicKey() const;
    seal::RelinKeys getRelinKeys() const;
    seal::GaloisKeys getGaloisKeys() const;
    
    std::vector<uint8_t> getSerializedParms() const;
    std::vector<uint8_t> getSerializedPk() const;
    std::vector<uint8_t> getSerializedRlk() const;
    std::vector<uint8_t> getSerializedGalk() const;

private:
    void createParms();
    void createKeys();

    double mScale;
    std::shared_ptr<seal::SEALContext> mpContext;
    
    seal::SecretKey mSecretKey;
    seal::PublicKey mPublicKey;
    seal::RelinKeys mRelinKeys;
    seal::GaloisKeys mGaloisKeys;
    
    std::vector<uint8_t> mSerializedParms;
    std::vector<uint8_t> mSerializedPk;
    std::vector<uint8_t> mSerializedRlk;
    std::vector<uint8_t> mSerializedGalk;
};

#endif // SXR_PARMS_AND_KEYS_HPP_

