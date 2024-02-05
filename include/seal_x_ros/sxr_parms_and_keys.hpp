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

    double get_scale() const;
    
    seal::SecretKey get_secret_key() const;
    seal::PublicKey get_public_key() const;
    seal::RelinKeys get_relin_keys() const;
    seal::GaloisKeys get_galois_keys() const;
    
    std::vector<uint8_t> get_serialized_parms() const;
    std::vector<uint8_t> get_serialized_pk() const;
    std::vector<uint8_t> get_serialized_rlk() const;
    std::vector<uint8_t> get_serialized_galk() const;

private:
    void create_parms();
    void create_keys();

    double scale_;
    std::shared_ptr<seal::SEALContext> context_;
    
    seal::SecretKey secret_key_;
    seal::PublicKey public_key_;
    seal::RelinKeys relin_keys_;
    seal::GaloisKeys galois_keys_;
    
    std::vector<uint8_t> serialized_parms_;
    std::vector<uint8_t> serialized_pk_;
    std::vector<uint8_t> serialized_rlk_;
    std::vector<uint8_t> serialized_galk_;
};

#endif // SXR_PARMS_AND_KEYS_HPP_

