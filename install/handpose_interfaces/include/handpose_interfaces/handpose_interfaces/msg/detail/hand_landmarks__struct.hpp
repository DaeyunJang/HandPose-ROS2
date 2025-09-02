// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from handpose_interfaces:msg/HandLandmarks.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__STRUCT_HPP_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__handpose_interfaces__msg__HandLandmarks __attribute__((deprecated))
#else
# define DEPRECATED__handpose_interfaces__msg__HandLandmarks __declspec(deprecated)
#endif

namespace handpose_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HandLandmarks_
{
  using Type = HandLandmarks_<ContainerAllocator>;

  explicit HandLandmarks_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->label = "";
      this->score = 0.0f;
      this->width = 0ul;
      this->height = 0ul;
      this->handed_index = 0l;
    }
  }

  explicit HandLandmarks_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    label(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->label = "";
      this->score = 0.0f;
      this->width = 0ul;
      this->height = 0ul;
      this->handed_index = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    int8_t;
  _id_type id;
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _label_type label;
  using _score_type =
    float;
  _score_type score;
  using _width_type =
    uint32_t;
  _width_type width;
  using _height_type =
    uint32_t;
  _height_type height;
  using _landmarks_norm_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _landmarks_norm_type landmarks_norm;
  using _landmarks_canon_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _landmarks_canon_type landmarks_canon;
  using _landmarks_world_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _landmarks_world_type landmarks_world;
  using _handed_index_type =
    int32_t;
  _handed_index_type handed_index;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const int8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__score(
    const float & _arg)
  {
    this->score = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__landmarks_norm(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->landmarks_norm = _arg;
    return *this;
  }
  Type & set__landmarks_canon(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->landmarks_canon = _arg;
    return *this;
  }
  Type & set__landmarks_world(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->landmarks_world = _arg;
    return *this;
  }
  Type & set__handed_index(
    const int32_t & _arg)
  {
    this->handed_index = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    handpose_interfaces::msg::HandLandmarks_<ContainerAllocator> *;
  using ConstRawPtr =
    const handpose_interfaces::msg::HandLandmarks_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__handpose_interfaces__msg__HandLandmarks
    std::shared_ptr<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__handpose_interfaces__msg__HandLandmarks
    std::shared_ptr<handpose_interfaces::msg::HandLandmarks_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HandLandmarks_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->label != other.label) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->landmarks_norm != other.landmarks_norm) {
      return false;
    }
    if (this->landmarks_canon != other.landmarks_canon) {
      return false;
    }
    if (this->landmarks_world != other.landmarks_world) {
      return false;
    }
    if (this->handed_index != other.handed_index) {
      return false;
    }
    return true;
  }
  bool operator!=(const HandLandmarks_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HandLandmarks_

// alias to use template instance with default allocator
using HandLandmarks =
  handpose_interfaces::msg::HandLandmarks_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace handpose_interfaces

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__STRUCT_HPP_
