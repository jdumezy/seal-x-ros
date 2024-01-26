// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from seal_msgs:msg/SealData.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__MSG__DETAIL__SEAL_DATA__TRAITS_HPP_
#define SEAL_MSGS__MSG__DETAIL__SEAL_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "seal_msgs/msg/detail/seal_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace seal_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SealData & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SealData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SealData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace seal_msgs

namespace rosidl_generator_traits
{

[[deprecated("use seal_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const seal_msgs::msg::SealData & msg,
  std::ostream & out, size_t indentation = 0)
{
  seal_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use seal_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const seal_msgs::msg::SealData & msg)
{
  return seal_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<seal_msgs::msg::SealData>()
{
  return "seal_msgs::msg::SealData";
}

template<>
inline const char * name<seal_msgs::msg::SealData>()
{
  return "seal_msgs/msg/SealData";
}

template<>
struct has_fixed_size<seal_msgs::msg::SealData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<seal_msgs::msg::SealData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<seal_msgs::msg::SealData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SEAL_MSGS__MSG__DETAIL__SEAL_DATA__TRAITS_HPP_
