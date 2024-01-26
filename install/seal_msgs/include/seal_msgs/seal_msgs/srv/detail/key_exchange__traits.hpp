// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from seal_msgs:srv/KeyExchange.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__TRAITS_HPP_
#define SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "seal_msgs/srv/detail/key_exchange__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace seal_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const KeyExchange_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: public_key
  {
    out << "public_key: ";
    rosidl_generator_traits::value_to_yaml(msg.public_key, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const KeyExchange_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: public_key
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "public_key: ";
    rosidl_generator_traits::value_to_yaml(msg.public_key, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const KeyExchange_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace seal_msgs

namespace rosidl_generator_traits
{

[[deprecated("use seal_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const seal_msgs::srv::KeyExchange_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  seal_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use seal_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const seal_msgs::srv::KeyExchange_Request & msg)
{
  return seal_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<seal_msgs::srv::KeyExchange_Request>()
{
  return "seal_msgs::srv::KeyExchange_Request";
}

template<>
inline const char * name<seal_msgs::srv::KeyExchange_Request>()
{
  return "seal_msgs/srv/KeyExchange_Request";
}

template<>
struct has_fixed_size<seal_msgs::srv::KeyExchange_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<seal_msgs::srv::KeyExchange_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<seal_msgs::srv::KeyExchange_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace seal_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const KeyExchange_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const KeyExchange_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const KeyExchange_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace seal_msgs

namespace rosidl_generator_traits
{

[[deprecated("use seal_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const seal_msgs::srv::KeyExchange_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  seal_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use seal_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const seal_msgs::srv::KeyExchange_Response & msg)
{
  return seal_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<seal_msgs::srv::KeyExchange_Response>()
{
  return "seal_msgs::srv::KeyExchange_Response";
}

template<>
inline const char * name<seal_msgs::srv::KeyExchange_Response>()
{
  return "seal_msgs/srv/KeyExchange_Response";
}

template<>
struct has_fixed_size<seal_msgs::srv::KeyExchange_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<seal_msgs::srv::KeyExchange_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<seal_msgs::srv::KeyExchange_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<seal_msgs::srv::KeyExchange>()
{
  return "seal_msgs::srv::KeyExchange";
}

template<>
inline const char * name<seal_msgs::srv::KeyExchange>()
{
  return "seal_msgs/srv/KeyExchange";
}

template<>
struct has_fixed_size<seal_msgs::srv::KeyExchange>
  : std::integral_constant<
    bool,
    has_fixed_size<seal_msgs::srv::KeyExchange_Request>::value &&
    has_fixed_size<seal_msgs::srv::KeyExchange_Response>::value
  >
{
};

template<>
struct has_bounded_size<seal_msgs::srv::KeyExchange>
  : std::integral_constant<
    bool,
    has_bounded_size<seal_msgs::srv::KeyExchange_Request>::value &&
    has_bounded_size<seal_msgs::srv::KeyExchange_Response>::value
  >
{
};

template<>
struct is_service<seal_msgs::srv::KeyExchange>
  : std::true_type
{
};

template<>
struct is_service_request<seal_msgs::srv::KeyExchange_Request>
  : std::true_type
{
};

template<>
struct is_service_response<seal_msgs::srv::KeyExchange_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__TRAITS_HPP_
