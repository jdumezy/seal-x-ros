#ifndef SXR_CIPHERTEXT_HPP
#define SXR_CIPHERTEXT_HPP

#include "seal/seal.h"

#include "seal_x_ros/sxr_lib.hpp"

#include <vector>
#include <memory>
#include <cstdint>

class SXRCiphertext {
public:
	SXRCiphertext(seal::Ciphertext ciphertext);
	
	seal::Ciphertext get_ct();
	int get_depth();
	void set_ct(seal::Ciphertext new_ct);
	void set_depth(int new_depth);

private:
	seal::Ciphertext ciphertext_;	
	int depth_;
//	std::vector<double> primeArray_;
};

#endif // SXR_CIPHERTEXT_HPP

