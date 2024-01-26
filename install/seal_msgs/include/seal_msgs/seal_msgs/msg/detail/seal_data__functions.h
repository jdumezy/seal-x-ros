// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from seal_msgs:msg/SealData.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__MSG__DETAIL__SEAL_DATA__FUNCTIONS_H_
#define SEAL_MSGS__MSG__DETAIL__SEAL_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "seal_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "seal_msgs/msg/detail/seal_data__struct.h"

/// Initialize msg/SealData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * seal_msgs__msg__SealData
 * )) before or use
 * seal_msgs__msg__SealData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__msg__SealData__init(seal_msgs__msg__SealData * msg);

/// Finalize msg/SealData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__msg__SealData__fini(seal_msgs__msg__SealData * msg);

/// Create msg/SealData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * seal_msgs__msg__SealData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
seal_msgs__msg__SealData *
seal_msgs__msg__SealData__create();

/// Destroy msg/SealData message.
/**
 * It calls
 * seal_msgs__msg__SealData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__msg__SealData__destroy(seal_msgs__msg__SealData * msg);

/// Check for msg/SealData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__msg__SealData__are_equal(const seal_msgs__msg__SealData * lhs, const seal_msgs__msg__SealData * rhs);

/// Copy a msg/SealData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__msg__SealData__copy(
  const seal_msgs__msg__SealData * input,
  seal_msgs__msg__SealData * output);

/// Initialize array of msg/SealData messages.
/**
 * It allocates the memory for the number of elements and calls
 * seal_msgs__msg__SealData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__msg__SealData__Sequence__init(seal_msgs__msg__SealData__Sequence * array, size_t size);

/// Finalize array of msg/SealData messages.
/**
 * It calls
 * seal_msgs__msg__SealData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__msg__SealData__Sequence__fini(seal_msgs__msg__SealData__Sequence * array);

/// Create array of msg/SealData messages.
/**
 * It allocates the memory for the array and calls
 * seal_msgs__msg__SealData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
seal_msgs__msg__SealData__Sequence *
seal_msgs__msg__SealData__Sequence__create(size_t size);

/// Destroy array of msg/SealData messages.
/**
 * It calls
 * seal_msgs__msg__SealData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__msg__SealData__Sequence__destroy(seal_msgs__msg__SealData__Sequence * array);

/// Check for msg/SealData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__msg__SealData__Sequence__are_equal(const seal_msgs__msg__SealData__Sequence * lhs, const seal_msgs__msg__SealData__Sequence * rhs);

/// Copy an array of msg/SealData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__msg__SealData__Sequence__copy(
  const seal_msgs__msg__SealData__Sequence * input,
  seal_msgs__msg__SealData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SEAL_MSGS__MSG__DETAIL__SEAL_DATA__FUNCTIONS_H_
