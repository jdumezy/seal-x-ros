// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from seal_msgs:srv/KeyExchange.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__FUNCTIONS_H_
#define SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "seal_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "seal_msgs/srv/detail/key_exchange__struct.h"

/// Initialize srv/KeyExchange message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * seal_msgs__srv__KeyExchange_Request
 * )) before or use
 * seal_msgs__srv__KeyExchange_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__srv__KeyExchange_Request__init(seal_msgs__srv__KeyExchange_Request * msg);

/// Finalize srv/KeyExchange message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__srv__KeyExchange_Request__fini(seal_msgs__srv__KeyExchange_Request * msg);

/// Create srv/KeyExchange message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * seal_msgs__srv__KeyExchange_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
seal_msgs__srv__KeyExchange_Request *
seal_msgs__srv__KeyExchange_Request__create();

/// Destroy srv/KeyExchange message.
/**
 * It calls
 * seal_msgs__srv__KeyExchange_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__srv__KeyExchange_Request__destroy(seal_msgs__srv__KeyExchange_Request * msg);

/// Check for srv/KeyExchange message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__srv__KeyExchange_Request__are_equal(const seal_msgs__srv__KeyExchange_Request * lhs, const seal_msgs__srv__KeyExchange_Request * rhs);

/// Copy a srv/KeyExchange message.
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
seal_msgs__srv__KeyExchange_Request__copy(
  const seal_msgs__srv__KeyExchange_Request * input,
  seal_msgs__srv__KeyExchange_Request * output);

/// Initialize array of srv/KeyExchange messages.
/**
 * It allocates the memory for the number of elements and calls
 * seal_msgs__srv__KeyExchange_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__srv__KeyExchange_Request__Sequence__init(seal_msgs__srv__KeyExchange_Request__Sequence * array, size_t size);

/// Finalize array of srv/KeyExchange messages.
/**
 * It calls
 * seal_msgs__srv__KeyExchange_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__srv__KeyExchange_Request__Sequence__fini(seal_msgs__srv__KeyExchange_Request__Sequence * array);

/// Create array of srv/KeyExchange messages.
/**
 * It allocates the memory for the array and calls
 * seal_msgs__srv__KeyExchange_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
seal_msgs__srv__KeyExchange_Request__Sequence *
seal_msgs__srv__KeyExchange_Request__Sequence__create(size_t size);

/// Destroy array of srv/KeyExchange messages.
/**
 * It calls
 * seal_msgs__srv__KeyExchange_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__srv__KeyExchange_Request__Sequence__destroy(seal_msgs__srv__KeyExchange_Request__Sequence * array);

/// Check for srv/KeyExchange message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__srv__KeyExchange_Request__Sequence__are_equal(const seal_msgs__srv__KeyExchange_Request__Sequence * lhs, const seal_msgs__srv__KeyExchange_Request__Sequence * rhs);

/// Copy an array of srv/KeyExchange messages.
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
seal_msgs__srv__KeyExchange_Request__Sequence__copy(
  const seal_msgs__srv__KeyExchange_Request__Sequence * input,
  seal_msgs__srv__KeyExchange_Request__Sequence * output);

/// Initialize srv/KeyExchange message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * seal_msgs__srv__KeyExchange_Response
 * )) before or use
 * seal_msgs__srv__KeyExchange_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__srv__KeyExchange_Response__init(seal_msgs__srv__KeyExchange_Response * msg);

/// Finalize srv/KeyExchange message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__srv__KeyExchange_Response__fini(seal_msgs__srv__KeyExchange_Response * msg);

/// Create srv/KeyExchange message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * seal_msgs__srv__KeyExchange_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
seal_msgs__srv__KeyExchange_Response *
seal_msgs__srv__KeyExchange_Response__create();

/// Destroy srv/KeyExchange message.
/**
 * It calls
 * seal_msgs__srv__KeyExchange_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__srv__KeyExchange_Response__destroy(seal_msgs__srv__KeyExchange_Response * msg);

/// Check for srv/KeyExchange message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__srv__KeyExchange_Response__are_equal(const seal_msgs__srv__KeyExchange_Response * lhs, const seal_msgs__srv__KeyExchange_Response * rhs);

/// Copy a srv/KeyExchange message.
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
seal_msgs__srv__KeyExchange_Response__copy(
  const seal_msgs__srv__KeyExchange_Response * input,
  seal_msgs__srv__KeyExchange_Response * output);

/// Initialize array of srv/KeyExchange messages.
/**
 * It allocates the memory for the number of elements and calls
 * seal_msgs__srv__KeyExchange_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__srv__KeyExchange_Response__Sequence__init(seal_msgs__srv__KeyExchange_Response__Sequence * array, size_t size);

/// Finalize array of srv/KeyExchange messages.
/**
 * It calls
 * seal_msgs__srv__KeyExchange_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__srv__KeyExchange_Response__Sequence__fini(seal_msgs__srv__KeyExchange_Response__Sequence * array);

/// Create array of srv/KeyExchange messages.
/**
 * It allocates the memory for the array and calls
 * seal_msgs__srv__KeyExchange_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
seal_msgs__srv__KeyExchange_Response__Sequence *
seal_msgs__srv__KeyExchange_Response__Sequence__create(size_t size);

/// Destroy array of srv/KeyExchange messages.
/**
 * It calls
 * seal_msgs__srv__KeyExchange_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
void
seal_msgs__srv__KeyExchange_Response__Sequence__destroy(seal_msgs__srv__KeyExchange_Response__Sequence * array);

/// Check for srv/KeyExchange message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_seal_msgs
bool
seal_msgs__srv__KeyExchange_Response__Sequence__are_equal(const seal_msgs__srv__KeyExchange_Response__Sequence * lhs, const seal_msgs__srv__KeyExchange_Response__Sequence * rhs);

/// Copy an array of srv/KeyExchange messages.
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
seal_msgs__srv__KeyExchange_Response__Sequence__copy(
  const seal_msgs__srv__KeyExchange_Response__Sequence * input,
  seal_msgs__srv__KeyExchange_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__FUNCTIONS_H_
