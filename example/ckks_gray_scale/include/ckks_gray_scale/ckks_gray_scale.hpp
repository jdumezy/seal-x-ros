// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_
#define EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_

#include "seal_x_ros/sxr_encryptor.hpp"
#include "seal_x_ros/sxr_evaluator.hpp"

SXRCiphertext gray_scale(SXRCiphertext ciphertext,
                         SXREncryptor encryptor,
                         SXREvaluator evaluator);

#endif  // EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_

