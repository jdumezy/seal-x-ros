// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "webcam_msg/webcam_msg_lib.hpp"

std::vector<float> byteArrayToFloatArray(const std::vector<uint8_t>& byteArray) {
  std::vector<float> floatArray;
  size_t len = byteArray.size() / sizeof(float);
  floatArray.resize(len);

  for (size_t i = 0; i < len; ++i) {
    std::memcpy(&floatArray[i], &byteArray[i * sizeof(float)], sizeof(float));
  }

  return floatArray;
}

std::vector<uint8_t> floatArrayToByteArray(const std::vector<float>& floatArray) {
  std::vector<uint8_t> byteArray;
  byteArray.reserve(floatArray.size() * sizeof(float));

  for (const float& value : floatArray) {
    uint8_t temp[sizeof(float)];
    std::memcpy(temp, &value, sizeof(float));

    byteArray.insert(byteArray.end(), temp, temp + sizeof(float));
  }

  return byteArray;
}

std::vector<float> imageToFloatArray(cv::Mat image) {
  image.convertTo(image, CV_32FC3);

  std::vector<float> floatArray;
  floatArray.reserve(image.rows * image.cols * image.channels());

  for (int row = 0; row < image.rows; row++) {
    for (int col = 0; col < image.cols; col++) {
      cv::Vec3f intensity = image.at<cv::Vec3f>(row, col);
      floatArray.push_back(intensity.val[0]);
      floatArray.push_back(intensity.val[1]);
      floatArray.push_back(intensity.val[2]);
    }
  }

  return floatArray;
}

std::tuple<std::vector<float>, int, int> loadImage(const char* filename) {
  cv::Mat image = cv::imread(filename);
  if (image.empty()) {
    throw std::invalid_argument("Invalid filename");
  }

  return std::make_tuple(imageToFloatArray(image), image.cols, image.rows);
}

std::tuple<std::vector<float>, int, int> acquireWebcam() {
  cv::VideoCapture cap(0);
  if (!cap.isOpened()) {
    throw std::invalid_argument("Couldn't start acquisition");
  }
  
  cv::Mat image;
  cap >> image;
  
  if (image.empty()) {
    throw std::invalid_argument("Empty frame");
  }
  
  cap.release();
  
  return std::make_tuple(imageToFloatArray(image), image.cols, image.rows);
}

std::vector<float> transformVector(const std::vector<float>& input) {
  std::vector<float> output = input;
  for (size_t i = 0; i < input.size() - 2; i += 3) {
    output[i + 1] = input[i];
    output[i + 2] = input[i];
  }
  return output;
}

cv::Mat floatArrayToImage(std::vector<float> floatArray, int width, int height) {
  cv::Mat image(height, width, CV_32FC3, floatArray.data());
  image.convertTo(image, CV_8UC3);
  return image;
}

void displayFloatArray(std::vector<float> floatArray, int width, int height, const char* windowName) {
  cv::imshow(windowName, floatArrayToImage(floatArray, width, height));
  cv::waitKey(30);
}

void saveImage(cv::Mat image, const char* filename) {
  cv::imwrite(filename, image);
}

