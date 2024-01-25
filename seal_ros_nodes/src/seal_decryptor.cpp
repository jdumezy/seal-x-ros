#include "seal_ros_nodes/seal_decryptor.hpp"
#include "seal/seal.h"

//Decryptor::Decryptor(const std::shared_ptr<seal::SEALContext>& context, const seal::SecretKey& secret_key)
//    : context_(context), secret_key_(secret_key), decryptor_(context, secret_key) {
//}

//seal::Plaintext Decryptor::decrypt(const seal::Ciphertext& ciphertext) const {
//    seal::Plaintext plaintext;
//    decryptor_.decrypt(ciphertext, plaintext);
//    return plaintext;
//}


	ContextManager ctxManager;
	seal::Decryptor decryptor(context, secret_key);
