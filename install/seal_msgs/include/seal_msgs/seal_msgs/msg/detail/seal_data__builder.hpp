// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from seal_msgs:msg/SealData.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__MSG__DETAIL__SEAL_DATA__BUILDER_HPP_
#define SEAL_MSGS__MSG__DETAIL__SEAL_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "seal_msgs/msg/detail/seal_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace seal_msgs
{

namespace msg
{

namespace builder
{

class Init_SealData_data
{
public:
  Init_SealData_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::seal_msgs::msg::SealData data(::seal_msgs::msg::SealData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::seal_msgs::msg::SealData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::seal_msgs::msg::SealData>()
{
  return seal_msgs::msg::builder::Init_SealData_data();
}

}  // namespace seal_msgs

#endif  // SEAL_MSGS__MSG__DETAIL__SEAL_DATA__BUILDER_HPP_
