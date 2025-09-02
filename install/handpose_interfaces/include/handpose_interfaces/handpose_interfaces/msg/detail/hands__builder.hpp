// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from handpose_interfaces:msg/Hands.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__BUILDER_HPP_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "handpose_interfaces/msg/detail/hands__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace handpose_interfaces
{

namespace msg
{

namespace builder
{

class Init_Hands_hands
{
public:
  explicit Init_Hands_hands(::handpose_interfaces::msg::Hands & msg)
  : msg_(msg)
  {}
  ::handpose_interfaces::msg::Hands hands(::handpose_interfaces::msg::Hands::_hands_type arg)
  {
    msg_.hands = std::move(arg);
    return std::move(msg_);
  }

private:
  ::handpose_interfaces::msg::Hands msg_;
};

class Init_Hands_header
{
public:
  Init_Hands_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Hands_hands header(::handpose_interfaces::msg::Hands::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Hands_hands(msg_);
  }

private:
  ::handpose_interfaces::msg::Hands msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::handpose_interfaces::msg::Hands>()
{
  return handpose_interfaces::msg::builder::Init_Hands_header();
}

}  // namespace handpose_interfaces

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__BUILDER_HPP_
