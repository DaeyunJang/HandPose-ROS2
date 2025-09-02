// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from handpose_interfaces:msg/HandLandmarks.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__TRAITS_HPP_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "handpose_interfaces/msg/detail/hand_landmarks__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace handpose_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const HandLandmarks & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << ", ";
  }

  // member: score
  {
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: landmarks_norm
  {
    if (msg.landmarks_norm.size() == 0) {
      out << "landmarks_norm: []";
    } else {
      out << "landmarks_norm: [";
      size_t pending_items = msg.landmarks_norm.size();
      for (auto item : msg.landmarks_norm) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: landmarks_canon
  {
    if (msg.landmarks_canon.size() == 0) {
      out << "landmarks_canon: []";
    } else {
      out << "landmarks_canon: [";
      size_t pending_items = msg.landmarks_canon.size();
      for (auto item : msg.landmarks_canon) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: landmarks_world
  {
    if (msg.landmarks_world.size() == 0) {
      out << "landmarks_world: []";
    } else {
      out << "landmarks_world: [";
      size_t pending_items = msg.landmarks_world.size();
      for (auto item : msg.landmarks_world) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: handed_index
  {
    out << "handed_index: ";
    rosidl_generator_traits::value_to_yaml(msg.handed_index, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HandLandmarks & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }

  // member: score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: landmarks_norm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.landmarks_norm.size() == 0) {
      out << "landmarks_norm: []\n";
    } else {
      out << "landmarks_norm:\n";
      for (auto item : msg.landmarks_norm) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: landmarks_canon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.landmarks_canon.size() == 0) {
      out << "landmarks_canon: []\n";
    } else {
      out << "landmarks_canon:\n";
      for (auto item : msg.landmarks_canon) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: landmarks_world
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.landmarks_world.size() == 0) {
      out << "landmarks_world: []\n";
    } else {
      out << "landmarks_world:\n";
      for (auto item : msg.landmarks_world) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: handed_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "handed_index: ";
    rosidl_generator_traits::value_to_yaml(msg.handed_index, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HandLandmarks & msg, bool use_flow_style = false)
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
  const handpose_interfaces::msg::HandLandmarks & msg,
  std::ostream & out, size_t indentation = 0)
{
  handpose_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use handpose_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const handpose_interfaces::msg::HandLandmarks & msg)
{
  return handpose_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<handpose_interfaces::msg::HandLandmarks>()
{
  return "handpose_interfaces::msg::HandLandmarks";
}

template<>
inline const char * name<handpose_interfaces::msg::HandLandmarks>()
{
  return "handpose_interfaces/msg/HandLandmarks";
}

template<>
struct has_fixed_size<handpose_interfaces::msg::HandLandmarks>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<handpose_interfaces::msg::HandLandmarks>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<handpose_interfaces::msg::HandLandmarks>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__TRAITS_HPP_
