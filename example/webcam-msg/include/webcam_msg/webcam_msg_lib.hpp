// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_WEBCAM_MSG_WEBCAM_MSG_LIB_HPP_
#define INCLUDE_WEBCAM_MSG_WEBCAM_MSG_LIB_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <stdexcept>

#define WIDTH 320
#define HEIGHT 240

std::vector<float> byteArrayToFloatArray(const std::vector<uint8_t>& byteArray);
std::vector<uint8_t> floatArrayToByteArray(const std::vector<float>& floatArray);

std::tuple<std::vector<float>, int, int> loadImage(const char* filename);
std::tuple<std::vector<float>, int, int> acquireWebcam();
void displayFloatArray(std::vector<float> floatArray, int width, int height, const char* windowName);
void saveImage(cv::Mat image, const char* filename);

#endif  // INCLUDE_WEBCAM_MSG_WEBCAM_MSG_LIB_HPP_

