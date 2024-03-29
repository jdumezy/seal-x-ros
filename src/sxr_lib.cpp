// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#include "seal_x_ros/sxr_lib.hpp"

double calculateScale(int depth, double scale, std::vector<double> primeArray) {
  double newScale = scale;
  int maxDepth = static_cast<int>(primeArray.size());

  if (depth == 0) {
    return newScale;
  } else if (depth < 0) {
    throw std::invalid_argument("Depth must be positive");
  } else if (depth > maxDepth) {
    throw std::invalid_argument("Depth is too deep for coeff modulus");
  } else {
    for (int i = 0; i < depth; i++) {
      newScale *= pow(scale, pow(2, (depth - i - 1))) / pow(primeArray[i], pow(2, (depth - i - 1)));
    }
    return newScale;
  }
}

int calculateDepth(double ciphertextScale, double scale,
                   std::vector<double> primeArray) {
  int depth = 0;
  double calculatedScale = scale;
  int maxDepth = static_cast<int>(primeArray.size());

  for (int i = 0; i < maxDepth; i++) {
    if (std::abs(calculatedScale - ciphertextScale) < 0.0001) {
      return depth;
    }
    depth++;
    calculatedScale = calculateScale(depth, scale, primeArray);
  }

  if (depth == maxDepth) {
    throw std::runtime_error("Unable to match the ciphertext scale");
  }
  return -1;
}

std::vector<std::vector<float>> splitMessage(const std::vector<float>& message,
                                             size_t chunkSize) {
  chunkSize -= (chunkSize % 3);  // working with rgb images
  std::vector<std::vector<float>> chunks;
  for (size_t i = 0; i < message.size(); i += chunkSize) {
    std::vector<float> chunk(message.begin() + i,
                             message.begin() + std::min(i + chunkSize, message.size()));
    chunks.push_back(chunk);
  }
  return chunks;
}

std::vector<float> glueMessage(const std::vector<std::vector<float>>& messageArray) {
  std::vector<float> message;
  for (const std::vector<float>& msg : messageArray) {
    message.insert(message.end(), msg.begin(), msg.end());
  }
  return message;
}

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

std::vector<double> floatArrayToDoubleArray(const std::vector<float>& floatArray) {
  std::vector<double> doubleArray;
  doubleArray.reserve(floatArray.size());
  for (float value : floatArray) {
    doubleArray.push_back(static_cast<double>(value));
  }
  return doubleArray;
}

std::vector<float> doubleArrayToFloatArray(const std::vector<double>& doubleArray) {
  std::vector<float> floatArray;
  floatArray.reserve(doubleArray.size());
  for (double value : doubleArray) {
    floatArray.push_back(static_cast<float>(value));
  }
  return floatArray;
}

seal::EncryptionParameters deserializeToParms(std::vector<uint8_t> serializedParms) {
  seal::EncryptionParameters parms;
  parms.load(reinterpret_cast<const seal::seal_byte*>(serializedParms.data()),
             serializedParms.size());
  return parms;
}

seal::PublicKey deserializeToPk(std::vector<uint8_t> serializedPk,
                                seal::SEALContext* pContext) {
  seal::PublicKey pk;
  pk.load(*pContext,
          reinterpret_cast<const seal::seal_byte*>(serializedPk.data()),
          serializedPk.size());
  return pk;
}

seal::RelinKeys deserializeToRlk(std::vector<uint8_t> serializedRlk,
                                 seal::SEALContext* pContext) {
  seal::RelinKeys rlk;
  rlk.load(*pContext,
           reinterpret_cast<const seal::seal_byte*>(serializedRlk.data()),
           serializedRlk.size());
  return rlk;
}

seal::GaloisKeys deserializeToGalk(std::vector<uint8_t> serializedGalk,
                                   seal::SEALContext* pContext) {
  seal::GaloisKeys galk;
  galk.load(*pContext,
            reinterpret_cast<const seal::seal_byte*>(serializedGalk.data()),
            serializedGalk.size());
  return galk;
}

seal::Ciphertext deserializeToCt(std::vector<uint8_t> serializedCt,
                                 seal::SEALContext* pContext) {
  seal::Ciphertext ct;
  ct.load(*pContext,
          reinterpret_cast<const seal::seal_byte*>(serializedCt.data()),
          serializedCt.size());
  return ct;
}

Semaphore::Semaphore(int count = 0) : count_(count) {}

void Semaphore::release() {
  std::unique_lock<std::mutex> lock(mutex_);
  ++count_;
  cv_.notify_one();
}

void Semaphore::acquire() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [this] { return count_ > 0; });
  --count_;
}

