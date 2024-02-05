#include "seal_x_ros/sxr_lib.hpp"

double calculate_scale(int depth, double scale, std::vector<double> cm_prime_array) {
	double new_scale = scale;
	int max_depth = static_cast<int>(cm_prime_array.size());
	
	if (depth == 0) {
		return new_scale;
	}
	else if (depth < 0) {
		throw std::invalid_argument("Depth must be positive");
	}
	else if (depth > max_depth) {
		throw std::invalid_argument("Depth is too deep for coeff modulus");
	}
	else {
		for (int i = 0; i < depth; i++) {
			new_scale *= pow(scale, pow(2, (depth - i - 1))) / pow(cm_prime_array[i], pow(2, (depth - i - 1)));
		}
		return new_scale;
	}
}

int calculateDepth(double ciphertextScale, double scale, std::vector<double> primeArray) {
	int depth = 0;
	double calculatedScale = scale;
	
	int max_depth = static_cast<int>(primeArray.size());
	
	for (int i = 0; i < max_depth; i++) {
		if (std::abs(calculatedScale - ciphertextScale) < 0.0001) {
			return depth;
		}
		depth++;
		calculatedScale = calculate_scale(depth, scale, primeArray);
	}
	
	if (depth == max_depth) {
		throw std::runtime_error("Unable to match the ciphertext scale");
	}
	
	return -1;
}

std::vector<double> convert_float_array_to_double(const std::vector<float>& float_array) {
	std::vector<double> double_array;
	double_array.reserve(float_array.size());
	
	for (float value : float_array) {
		double_array.push_back(static_cast<double>(value));
	}
	
	return double_array;
}

std::vector<float> convert_double_array_to_float(const std::vector<double>& double_array) {
	std::vector<float> float_array;
	float_array.reserve(double_array.size());
	
	for (double value : double_array) {
		float_array.push_back(static_cast<float>(value));
	}
	
	return float_array;
}

std::shared_ptr<seal::SEALContext> context_from_parms(const std::vector<uint8_t>& serialized_parms) {
	seal::EncryptionParameters parms = deserialize_to_parms(serialized_parms);
	std::shared_ptr<seal::SEALContext> context = std::make_shared<seal::SEALContext>(parms);
	return context;
}

seal::EncryptionParameters deserialize_to_parms(std::vector<uint8_t> serialized_parms) {
	seal::EncryptionParameters parms;
	parms.load(reinterpret_cast<const seal::seal_byte*>(serialized_parms.data()), serialized_parms.size());
	return parms;
}

seal::PublicKey deserialize_to_pk(std::vector<uint8_t> serialized_pk, std::shared_ptr<seal::SEALContext> context) {
	seal::PublicKey pk;
	pk.load(*context, reinterpret_cast<const seal::seal_byte*>(serialized_pk.data()), serialized_pk.size());
	return pk;
}

seal::RelinKeys deserialize_to_rlk(std::vector<uint8_t> serialized_rlk, std::shared_ptr<seal::SEALContext> context) {
	seal::RelinKeys rlk;
	rlk.load(*context, reinterpret_cast<const seal::seal_byte*>(serialized_rlk.data()), serialized_rlk.size());
	return rlk;
}

seal::GaloisKeys deserialize_to_galk(std::vector<uint8_t> serialized_galk, std::shared_ptr<seal::SEALContext> context) {
	seal::GaloisKeys galk;
	galk.load(*context, reinterpret_cast<const seal::seal_byte*>(serialized_galk.data()), serialized_galk.size());
	return galk;
}

seal::Ciphertext deserialize_to_ct(std::vector<uint8_t> serialized_ct, std::shared_ptr<seal::SEALContext> context) {
	seal::Ciphertext ct;
	ct.load(*context, reinterpret_cast<const seal::seal_byte*>(serialized_ct.data()), serialized_ct.size());
	return ct;
}

