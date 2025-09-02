// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from handpose_interfaces:msg/Hands.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__STRUCT_HPP_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'hands'
#include "handpose_interfaces/msg/detail/hand_landmarks__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__handpose_interfaces__msg__Hands __attribute__((deprecated))
#else
# define DEPRECATED__handpose_interfaces__msg__Hands __declspec(deprecated)
#endif

namespace handpose_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Hands_
{
  using Type = Hands_<ContainerAllocator>;

  explicit Hands_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Hands_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _hands_type =
    std::vector<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>>>;
  _hands_type hands;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__hands(
    const std::vector<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>>> & _arg)
  {
    this->hands = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    handpose_interfaces::msg::Hands_<ContainerAllocator> *;
  using ConstRawPtr =
    const handpose_interfaces::msg::Hands_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<handpose_interfaces::msg::Hands_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<handpose_interfaces::msg::Hands_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      handpose_interfaces::msg::Hands_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<handpose_interfaces::msg::Hands_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      handpose_interfaces::msg::Hands_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<handpose_interfaces::msg::Hands_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<handpose_interfaces::msg::Hands_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<handpose_interfaces::msg::Hands_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__handpose_interfaces__msg__Hands
    std::shared_ptr<handpose_interfaces::msg::Hands_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__handpose_interfaces__msg__Hands
    std::shared_ptr<handpose_interfaces::msg::Hands_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Hands_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->hands != other.hands) {
      return false;
    }
    return true;
  }
  bool operator!=(const Hands_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Hands_

// alias to use template instance with default allocator
using Hands =
  handpose_interfaces::msg::Hands_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace handpose_interfaces

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__STRUCT_HPP_
