// Copyright 2024 Jules Dumezy
// This code is licensed under MIT license (see LICENSE.md for details)

#ifndef INCLUDE_SEAL_X_ROS_SXR_CIPHERTEXT_HPP_
#define INCLUDE_SEAL_X_ROS_SXR_CIPHERTEXT_HPP_

#include "seal/seal.h"

/**
 * @class SXRCiphertext
 * 
 * @brief A class that wraps a SEAL Ciphertext with additional information and functionality.
 * 
 * This class encapsulates a SEAL Ciphertext along with metadata such as its depth. 
 * It provides methods to get and set the ciphertext and its depth, allowing for 
 * easier management of encrypted data within the SEAL-X-ROS framework.
 */
class SXRCiphertext {
 public:
  /**
   * @brief Constructs a new SXRCiphertext object from a SEAL Ciphertext.
   *
   * Initializes the object with a given SEAL Ciphertext. The depth is initially set to 0.
   *
   * @param ciphertext The SEAL Ciphertext to encapsulate.
   */
  SXRCiphertext(seal::Ciphertext ciphertext);

  /**
   * @brief Retrieves the encapsulated SEAL Ciphertext.
   *
   * This method returns the SEAL Ciphertext stored within the object.
   *
   * @return seal::Ciphertext The stored SEAL Ciphertext.
   */
  seal::Ciphertext getCiphertext();

  /**
   * @brief Retrieves the depth of the encapsulated Ciphertext.
   *
   * This method returns the depth of the Ciphertext, which is relevant for tracking
   * the number of operations performed on it.
   *
   * @return int The depth of the Ciphertext.
   */
  int getDepth();

  /**
   * @brief Sets the encapsulated Ciphertext to a new value.
   *
   * This method allows for replacing the stored SEAL Ciphertext with a new one.
   *
   * @param newCiphertext The new SEAL Ciphertext to store.
   */
  void setCiphertext(seal::Ciphertext newCiphertext);

  /**
   * @brief Sets the depth of the encapsulated Ciphertext.
   *
   * This method allows for updating the depth of the Ciphertext, typically after
   * an operation that affects its depth.
   *
   * @param newDepth The new depth of the Ciphertext.
   */
  void setDepth(int newDepth);

 private:
  seal::Ciphertext mCiphertext;  ///< The SEAL Ciphertext being encapsulated.
  int mDepth;  ///< The depth of the Ciphertext, indicating the number of operations performed.
};

#endif  // INCLUDE_SEAL_X_ROS_SXR_CIPHERTEXT_HPP_

