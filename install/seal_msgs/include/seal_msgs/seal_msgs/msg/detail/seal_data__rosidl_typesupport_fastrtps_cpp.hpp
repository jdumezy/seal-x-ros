// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from seal_msgs:msg/SealData.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__MSG__DETAIL__SEAL_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SEAL_MSGS__MSG__DETAIL__SEAL_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "seal_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "seal_msgs/msg/detail/seal_data__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace seal_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_seal_msgs
cdr_serialize(
  const seal_msgs::msg::SealData & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_seal_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  seal_msgs::msg::SealData & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_seal_msgs
get_serialized_size(
  const seal_msgs::msg::SealData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_seal_msgs
max_serialized_size_SealData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace seal_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_seal_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, seal_msgs, msg, SealData)();

#ifdef __cplusplus
}
#endif

#endif  // SEAL_MSGS__MSG__DETAIL__SEAL_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
