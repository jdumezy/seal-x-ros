// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from seal_msgs:msg/SealData.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__MSG__DETAIL__SEAL_DATA__STRUCT_H_
#define SEAL_MSGS__MSG__DETAIL__SEAL_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/SealData in the package seal_msgs.
typedef struct seal_msgs__msg__SealData
{
  rosidl_runtime_c__uint8__Sequence data;
} seal_msgs__msg__SealData;

// Struct for a sequence of seal_msgs__msg__SealData.
typedef struct seal_msgs__msg__SealData__Sequence
{
  seal_msgs__msg__SealData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} seal_msgs__msg__SealData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SEAL_MSGS__MSG__DETAIL__SEAL_DATA__STRUCT_H_
