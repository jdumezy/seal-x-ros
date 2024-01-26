#ifndef SXR_LIB_HPP
#define SXR_LIB_HPP

#include "seal/seal.h"

#include <vector>
#include <memory>

std::shared_ptr<seal::SEALContext> CreateSEALContextFromParameters(const std::vector<uint8_t>& serialized_parms);

#endif // SXR_LIB_HPP

