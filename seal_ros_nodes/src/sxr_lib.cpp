#include "seal_ros_nodes/sxr_lib.hpp"

std::shared_ptr<seal::SEALContext> CreateSEALContextFromParameters(const std::vector<uint8_t>& serialized_parms) {
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

seal::Ciphertext deserialize_to_ct(std::vector<uint8_t> serialized_ct, std::shared_ptr<seal::SEALContext> context) {
	seal::Ciphertext ct;
	ct.load(*context, reinterpret_cast<const seal::seal_byte*>(serialized_ct.data()), serialized_ct.size());
	return ct;
}

