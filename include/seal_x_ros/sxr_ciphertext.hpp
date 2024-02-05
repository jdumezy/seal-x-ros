#ifndef SXR_CIPHERTEXT_HPP_
#define SXR_CIPHERTEXT_HPP_

#include "seal/seal.h"

class SXRCiphertext {
public:
    SXRCiphertext(seal::Ciphertext ciphertext);
    
    seal::Ciphertext getCiphertext();
    int getDepth();
    void setCiphertext(seal::Ciphertext newCiphertext);
    void setDepth(int newDepth);

private:
    seal::Ciphertext mCiphertext;
    int mDepth;
};

#endif // SXR_CIPHERTEXT_HPP_

