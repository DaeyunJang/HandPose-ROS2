// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from handpose_interfaces:msg/HandLandmarks.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__BUILDER_HPP_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "handpose_interfaces/msg/detail/hand_landmarks__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace handpose_interfaces
{

namespace msg
{

namespace builder
{

class Init_HandLandmarks_handed_index
{
public:
  explicit Init_HandLandmarks_handed_index(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  ::handpose_interfaces::msg::HandLandmarks handed_index(::handpose_interfaces::msg::HandLandmarks::_handed_index_type arg)
  {
    msg_.handed_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_landmarks_world
{
public:
  explicit Init_HandLandmarks_landmarks_world(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  Init_HandLandmarks_handed_index landmarks_world(::handpose_interfaces::msg::HandLandmarks::_landmarks_world_type arg)
  {
    msg_.landmarks_world = std::move(arg);
    return Init_HandLandmarks_handed_index(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_landmarks_canon
{
public:
  explicit Init_HandLandmarks_landmarks_canon(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  Init_HandLandmarks_landmarks_world landmarks_canon(::handpose_interfaces::msg::HandLandmarks::_landmarks_canon_type arg)
  {
    msg_.landmarks_canon = std::move(arg);
    return Init_HandLandmarks_landmarks_world(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_landmarks_norm
{
public:
  explicit Init_HandLandmarks_landmarks_norm(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  Init_HandLandmarks_landmarks_canon landmarks_norm(::handpose_interfaces::msg::HandLandmarks::_landmarks_norm_type arg)
  {
    msg_.landmarks_norm = std::move(arg);
    return Init_HandLandmarks_landmarks_canon(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_height
{
public:
  explicit Init_HandLandmarks_height(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  Init_HandLandmarks_landmarks_norm height(::handpose_interfaces::msg::HandLandmarks::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_HandLandmarks_landmarks_norm(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_width
{
public:
  explicit Init_HandLandmarks_width(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  Init_HandLandmarks_height width(::handpose_interfaces::msg::HandLandmarks::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_HandLandmarks_height(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_score
{
public:
  explicit Init_HandLandmarks_score(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  Init_HandLandmarks_width score(::handpose_interfaces::msg::HandLandmarks::_score_type arg)
  {
    msg_.score = std::move(arg);
    return Init_HandLandmarks_width(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_label
{
public:
  explicit Init_HandLandmarks_label(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  Init_HandLandmarks_score label(::handpose_interfaces::msg::HandLandmarks::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_HandLandmarks_score(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_id
{
public:
  explicit Init_HandLandmarks_id(::handpose_interfaces::msg::HandLandmarks & msg)
  : msg_(msg)
  {}
  Init_HandLandmarks_label id(::handpose_interfaces::msg::HandLandmarks::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_HandLandmarks_label(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

class Init_HandLandmarks_header
{
public:
  Init_HandLandmarks_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HandLandmarks_id header(::handpose_interfaces::msg::HandLandmarks::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HandLandmarks_id(msg_);
  }

private:
  ::handpose_interfaces::msg::HandLandmarks msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::handpose_interfaces::msg::HandLandmarks>()
{
  return handpose_interfaces::msg::builder::Init_HandLandmarks_header();
}

}  // namespace handpose_interfaces

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__BUILDER_HPP_
