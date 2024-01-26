// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from seal_msgs:srv/KeyExchange.idl
// generated code does not contain a copyright notice

#ifndef SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__STRUCT_HPP_
#define SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__seal_msgs__srv__KeyExchange_Request __attribute__((deprecated))
#else
# define DEPRECATED__seal_msgs__srv__KeyExchange_Request __declspec(deprecated)
#endif

namespace seal_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct KeyExchange_Request_
{
  using Type = KeyExchange_Request_<ContainerAllocator>;

  explicit KeyExchange_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->public_key = "";
    }
  }

  explicit KeyExchange_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : public_key(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->public_key = "";
    }
  }

  // field types and members
  using _public_key_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _public_key_type public_key;

  // setters for named parameter idiom
  Type & set__public_key(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->public_key = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    seal_msgs::srv::KeyExchange_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const seal_msgs::srv::KeyExchange_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<seal_msgs::srv::KeyExchange_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<seal_msgs::srv::KeyExchange_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      seal_msgs::srv::KeyExchange_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<seal_msgs::srv::KeyExchange_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      seal_msgs::srv::KeyExchange_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<seal_msgs::srv::KeyExchange_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<seal_msgs::srv::KeyExchange_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<seal_msgs::srv::KeyExchange_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__seal_msgs__srv__KeyExchange_Request
    std::shared_ptr<seal_msgs::srv::KeyExchange_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__seal_msgs__srv__KeyExchange_Request
    std::shared_ptr<seal_msgs::srv::KeyExchange_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyExchange_Request_ & other) const
  {
    if (this->public_key != other.public_key) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyExchange_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyExchange_Request_

// alias to use template instance with default allocator
using KeyExchange_Request =
  seal_msgs::srv::KeyExchange_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace seal_msgs


#ifndef _WIN32
# define DEPRECATED__seal_msgs__srv__KeyExchange_Response __attribute__((deprecated))
#else
# define DEPRECATED__seal_msgs__srv__KeyExchange_Response __declspec(deprecated)
#endif

namespace seal_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct KeyExchange_Response_
{
  using Type = KeyExchange_Response_<ContainerAllocator>;

  explicit KeyExchange_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit KeyExchange_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    seal_msgs::srv::KeyExchange_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const seal_msgs::srv::KeyExchange_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<seal_msgs::srv::KeyExchange_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<seal_msgs::srv::KeyExchange_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      seal_msgs::srv::KeyExchange_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<seal_msgs::srv::KeyExchange_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      seal_msgs::srv::KeyExchange_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<seal_msgs::srv::KeyExchange_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<seal_msgs::srv::KeyExchange_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<seal_msgs::srv::KeyExchange_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__seal_msgs__srv__KeyExchange_Response
    std::shared_ptr<seal_msgs::srv::KeyExchange_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__seal_msgs__srv__KeyExchange_Response
    std::shared_ptr<seal_msgs::srv::KeyExchange_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyExchange_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyExchange_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyExchange_Response_

// alias to use template instance with default allocator
using KeyExchange_Response =
  seal_msgs::srv::KeyExchange_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace seal_msgs

namespace seal_msgs
{

namespace srv
{

struct KeyExchange
{
  using Request = seal_msgs::srv::KeyExchange_Request;
  using Response = seal_msgs::srv::KeyExchange_Response;
};

}  // namespace srv

}  // namespace seal_msgs

#endif  // SEAL_MSGS__SRV__DETAIL__KEY_EXCHANGE__STRUCT_HPP_
