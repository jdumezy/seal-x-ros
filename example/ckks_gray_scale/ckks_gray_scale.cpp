// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "ckks_gray_scale/ckks_gray_scale.hpp"

SXRCiphertext gray_scale(SXRCiphertext ciphertext,
                         SXREvaluator* pEvaluator) {
  SXRCiphertext ciphertextOne = pEvaluator->rotateVector(ciphertext, 1);
  SXRCiphertext ciphertextTwo = pEvaluator->rotateVector(ciphertext, 2);

  SXRCiphertext firstSum = pEvaluator->add(ciphertext, ciphertextOne);
  SXRCiphertext secondSum = pEvaluator->add(firstSum, ciphertextTwo);

  SXRCiphertext result  = pEvaluator->multiplyFloat(secondSum, 0.3333333f);

  return result;
}

std::tuple<std::vector<float>, int, int> loadImage(const char* filename) {
  cv::Mat image = cv::imread(filename);
  if (image.empty()) {
    throw std::invalid_argument("Invalid filename");
  }

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

  return std::make_tuple(floatArray, image.cols, image.rows);
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

void saveImage(cv::Mat image, const char* filename) {
  cv::imwrite(filename, image);
}

