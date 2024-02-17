
// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_
#define EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <stdexcept>

#include "seal_x_ros/sxr_ciphertext.hpp"
#include "seal_x_ros/sxr_evaluator.hpp"

SXRCiphertext gray_scale(SXRCiphertext ciphertext,
                         SXREvaluator* evaluator);

std::tuple<std::vector<float>, int, int> loadImage(const char* filename);
cv::Mat floatArrayToImage(std::vector<float> floatArray, int width, int height);
void saveImage(cv::Mat image, const char* filename);

#endif  // EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_

