// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from seal_msgs:msg/SealData.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__MSG__DETAIL__SEAL_DATA__STRUCT_HPP_
#define SEAL_MSGS__MSG__DETAIL__SEAL_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__seal_msgs__msg__SealData __attribute__((deprecated))
#else
# define DEPRECATED__seal_msgs__msg__SealData __declspec(deprecated)
#endif

namespace seal_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SealData_
{
  using Type = SealData_<ContainerAllocator>;

  explicit SealData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SealData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    seal_msgs::msg::SealData_<ContainerAllocator> *;
  using ConstRawPtr =
    const seal_msgs::msg::SealData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<seal_msgs::msg::SealData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<seal_msgs::msg::SealData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      seal_msgs::msg::SealData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<seal_msgs::msg::SealData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      seal_msgs::msg::SealData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<seal_msgs::msg::SealData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<seal_msgs::msg::SealData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<seal_msgs::msg::SealData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__seal_msgs__msg__SealData
    std::shared_ptr<seal_msgs::msg::SealData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__seal_msgs__msg__SealData
    std::shared_ptr<seal_msgs::msg::SealData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SealData_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const SealData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SealData_

// alias to use template instance with default allocator
using SealData =
  seal_msgs::msg::SealData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace seal_msgs

#endif  // SEAL_MSGS__MSG__DETAIL__SEAL_DATA__STRUCT_HPP_
