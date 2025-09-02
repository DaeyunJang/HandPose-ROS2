// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from handpose_interfaces:msg/Hands.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__TRAITS_HPP_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "handpose_interfaces/msg/detail/hands__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'hands'
#include "handpose_interfaces/msg/detail/hand_landmarks__traits.hpp"

namespace handpose_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Hands & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: hands
  {
    if (msg.hands.size() == 0) {
      out << "hands: []";
    } else {
      out << "hands: [";
      size_t pending_items = msg.hands.size();
      for (auto item : msg.hands) {
        to_flow_style_yaml(item, out);
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
  const Hands & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: hands
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.hands.size() == 0) {
      out << "hands: []\n";
    } else {
      out << "hands:\n";
      for (auto item : msg.hands) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Hands & msg, bool use_flow_style = false)
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

}  // namespace handpose_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use handpose_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const handpose_interfaces::msg::Hands & msg,
  std::ostream & out, size_t indentation = 0)
{
  handpose_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use handpose_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const handpose_interfaces::msg::Hands & msg)
{
  return handpose_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<handpose_interfaces::msg::Hands>()
{
  return "handpose_interfaces::msg::Hands";
}

template<>
inline const char * name<handpose_interfaces::msg::Hands>()
{
  return "handpose_interfaces/msg/Hands";
}

template<>
struct has_fixed_size<handpose_interfaces::msg::Hands>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<handpose_interfaces::msg::Hands>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<handpose_interfaces::msg::Hands>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__TRAITS_HPP_
