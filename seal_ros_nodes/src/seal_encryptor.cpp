#include "seal_ros_nodes/seal_encryptor.hpp"
#include "seal/seal.h"

using namespace std;
using namespace seal;

//Encryptor::Encryptor(const std::shared_ptr<seal::SEALContext>& context, const seal::PublicKey& public_key)
//    : context_(context), public_key_(public_key), encryptor_(context, public_key) {
//}

//seal::Ciphertext Encryptor::encrypt(const seal::Plaintext& plaintext) const {
//    seal::Ciphertext ciphertext;
//    encryptor_.encrypt(plaintext, ciphertext);
//    return ciphertext;
//}

