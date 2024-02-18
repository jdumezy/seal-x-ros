
// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_
#define EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <stdexcept>

std::tuple<std::vector<float>, int, int> loadImage(const char* filename);
std::tuple<std::vector<float>, int, int> acquireWebcam();
cv::Mat floatArrayToImage(std::vector<float> floatArray, int width, int height);
void displayFloatArray(std::vector<float> floatArray, int width, int height, const char* windowName);
void saveImage(cv::Mat image, const char* filename);

#endif  // EXAMPLE_CKKS_GRAY_SCALE_INCLUDE_CKKS_GRAY_SCALE_CKKS_GRAY_SCALE_HPP_

