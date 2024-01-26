#include "seal_ros_nodes/sxr_lib.hpp"

std::shared_ptr<seal::SEALContext> CreateSEALContextFromParameters(const std::vector<uint8_t>& serialized_parms) {
	seal::EncryptionParameters parms;
	parms.load(reinterpret_cast<const seal::seal_byte*>(serialized_parms.data()), serialized_parms.size());
	std::shared_ptr<seal::SEALContext> context = std::make_shared<seal::SEALContext>(parms);
	return context;
}

