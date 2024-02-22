// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_SEAL_X_ROS_SXR_OPERATIONS_HPP_
#define INCLUDE_SEAL_X_ROS_SXR_OPERATIONS_HPP_

#include <vector>
#include <memory>
#include <cstdint>

#include "seal_x_ros/sxr_ciphertext.hpp"
#include "seal_x_ros/sxr_evaluator.hpp"

SXRCiphertext gray_scale(SXRCiphertext ciphertext,
                         SXREvaluator* pEvaluator);

#endif  // INCLUDE_SEAL_X_ROS_SXR_OPERATIONS_HPP_

