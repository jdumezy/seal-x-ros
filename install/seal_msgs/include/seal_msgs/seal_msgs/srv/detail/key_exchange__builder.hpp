// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from seal_msgs:srv/KeyExchange.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__BUILDER_HPP_
#define SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "seal_msgs/srv/detail/key_exchange__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace seal_msgs
{

namespace srv
{

namespace builder
{

class Init_KeyExchange_Request_public_key
{
public:
  Init_KeyExchange_Request_public_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::seal_msgs::srv::KeyExchange_Request public_key(::seal_msgs::srv::KeyExchange_Request::_public_key_type arg)
  {
    msg_.public_key = std::move(arg);
    return std::move(msg_);
  }

private:
  ::seal_msgs::srv::KeyExchange_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::seal_msgs::srv::KeyExchange_Request>()
{
  return seal_msgs::srv::builder::Init_KeyExchange_Request_public_key();
}

}  // namespace seal_msgs


namespace seal_msgs
{

namespace srv
{

namespace builder
{

class Init_KeyExchange_Response_message
{
public:
  explicit Init_KeyExchange_Response_message(::seal_msgs::srv::KeyExchange_Response & msg)
  : msg_(msg)
  {}
  ::seal_msgs::srv::KeyExchange_Response message(::seal_msgs::srv::KeyExchange_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::seal_msgs::srv::KeyExchange_Response msg_;
};

class Init_KeyExchange_Response_success
{
public:
  Init_KeyExchange_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KeyExchange_Response_message success(::seal_msgs::srv::KeyExchange_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_KeyExchange_Response_message(msg_);
  }

private:
  ::seal_msgs::srv::KeyExchange_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::seal_msgs::srv::KeyExchange_Response>()
{
  return seal_msgs::srv::builder::Init_KeyExchange_Response_success();
}

}  // namespace seal_msgs

#endif  // SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__BUILDER_HPP_
