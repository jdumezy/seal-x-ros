// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from seal_msgs:srv/KeyExchange.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__STRUCT_H_
#define SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'public_key'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/KeyExchange in the package seal_msgs.
typedef struct seal_msgs__srv__KeyExchange_Request
{
  rosidl_runtime_c__String public_key;
} seal_msgs__srv__KeyExchange_Request;

// Struct for a sequence of seal_msgs__srv__KeyExchange_Request.
typedef struct seal_msgs__srv__KeyExchange_Request__Sequence
{
  seal_msgs__srv__KeyExchange_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} seal_msgs__srv__KeyExchange_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/KeyExchange in the package seal_msgs.
typedef struct seal_msgs__srv__KeyExchange_Response
{
  bool success;
  rosidl_runtime_c__String message;
} seal_msgs__srv__KeyExchange_Response;

// Struct for a sequence of seal_msgs__srv__KeyExchange_Response.
typedef struct seal_msgs__srv__KeyExchange_Response__Sequence
{
  seal_msgs__srv__KeyExchange_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} seal_msgs__srv__KeyExchange_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__STRUCT_H_
