#include "seal_x_ros/sxr_lib.hpp"

double calculateScale(int depth, double scale, std::vector<double> primeArray) {
    double newScale = scale;
    int maxDepth = static_cast<int>(primeArray.size());
    
    if (depth == 0) {
        return newScale;
    }
    else if (depth < 0) {
        throw std::invalid_argument("Depth must be positive");
    }
    else if (depth > maxDepth) {
        throw std::invalid_argument("Depth is too deep for coeff modulus");
    }
    else {
        for (int i = 0; i < depth; i++) {
            newScale *= pow(scale, pow(2, (depth - i - 1))) / pow(primeArray[i], pow(2, (depth - i - 1)));
        }
        return newScale;
    }
}

int calculateDepth(double ciphertextScale, double scale, std::vector<double> primeArray) {
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

std::shared_ptr<seal::SEALContext> pContextFromParms(const std::vector<uint8_t>& serializedParms) {
    seal::EncryptionParameters parms = deserializeToParms(serializedParms);
    std::shared_ptr<seal::SEALContext> context = std::make_shared<seal::SEALContext>(parms);
    return context;
}

seal::EncryptionParameters deserializeToParms(std::vector<uint8_t> serializedParms) {
    seal::EncryptionParameters parms;
    parms.load(reinterpret_cast<const seal::seal_byte*>(serializedParms.data()), serializedParms.size());
    return parms;
}

seal::PublicKey deserializeToPk(std::vector<uint8_t> serializedPk, std::shared_ptr<seal::SEALContext> context) {
    seal::PublicKey pk;
    pk.load(*context, reinterpret_cast<const seal::seal_byte*>(serializedPk.data()), serializedPk.size());
    return pk;
}

seal::RelinKeys deserializeToRlk(std::vector<uint8_t> serializedRlk, std::shared_ptr<seal::SEALContext> context) {
    seal::RelinKeys rlk;
    rlk.load(*context, reinterpret_cast<const seal::seal_byte*>(serializedRlk.data()), serializedRlk.size());
    return rlk;
}

seal::GaloisKeys deserializeToGalk(std::vector<uint8_t> serializedGalk, std::shared_ptr<seal::SEALContext> context) {
    seal::GaloisKeys galk;
    galk.load(*context, reinterpret_cast<const seal::seal_byte*>(serializedGalk.data()), serializedGalk.size());
    return galk;
}

seal::Ciphertext deserializeToCt(std::vector<uint8_t> serializedCt, std::shared_ptr<seal::SEALContext> context) {
    seal::Ciphertext ct;
    ct.load(*context, reinterpret_cast<const seal::seal_byte*>(serializedCt.data()), serializedCt.size());
    return ct;
}

