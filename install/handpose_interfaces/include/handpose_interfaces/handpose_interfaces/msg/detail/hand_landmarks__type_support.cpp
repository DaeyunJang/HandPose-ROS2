// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from handpose_interfaces:msg/HandLandmarks.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "handpose_interfaces/msg/detail/hand_landmarks__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace handpose_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void HandLandmarks_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) handpose_interfaces::msg::HandLandmarks(_init);
}

void HandLandmarks_fini_function(void * message_memory)
{
  auto typed_message = static_cast<handpose_interfaces::msg::HandLandmarks *>(message_memory);
  typed_message->~HandLandmarks();
}

size_t size_function__HandLandmarks__landmarks_norm(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__HandLandmarks__landmarks_norm(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__HandLandmarks__landmarks_norm(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__HandLandmarks__landmarks_norm(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__HandLandmarks__landmarks_norm(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__HandLandmarks__landmarks_norm(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__HandLandmarks__landmarks_norm(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__HandLandmarks__landmarks_norm(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__HandLandmarks__landmarks_canon(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__HandLandmarks__landmarks_canon(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__HandLandmarks__landmarks_canon(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__HandLandmarks__landmarks_canon(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__HandLandmarks__landmarks_canon(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__HandLandmarks__landmarks_canon(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__HandLandmarks__landmarks_canon(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__HandLandmarks__landmarks_canon(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__HandLandmarks__landmarks_world(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__HandLandmarks__landmarks_world(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__HandLandmarks__landmarks_world(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__HandLandmarks__landmarks_world(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__HandLandmarks__landmarks_world(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__HandLandmarks__landmarks_world(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__HandLandmarks__landmarks_world(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__HandLandmarks__landmarks_world(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember HandLandmarks_message_member_array[10] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "label",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, label),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "score",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, score),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "landmarks_norm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, landmarks_norm),  // bytes offset in struct
    nullptr,  // default value
    size_function__HandLandmarks__landmarks_norm,  // size() function pointer
    get_const_function__HandLandmarks__landmarks_norm,  // get_const(index) function pointer
    get_function__HandLandmarks__landmarks_norm,  // get(index) function pointer
    fetch_function__HandLandmarks__landmarks_norm,  // fetch(index, &value) function pointer
    assign_function__HandLandmarks__landmarks_norm,  // assign(index, value) function pointer
    resize_function__HandLandmarks__landmarks_norm  // resize(index) function pointer
  },
  {
    "landmarks_canon",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, landmarks_canon),  // bytes offset in struct
    nullptr,  // default value
    size_function__HandLandmarks__landmarks_canon,  // size() function pointer
    get_const_function__HandLandmarks__landmarks_canon,  // get_const(index) function pointer
    get_function__HandLandmarks__landmarks_canon,  // get(index) function pointer
    fetch_function__HandLandmarks__landmarks_canon,  // fetch(index, &value) function pointer
    assign_function__HandLandmarks__landmarks_canon,  // assign(index, value) function pointer
    resize_function__HandLandmarks__landmarks_canon  // resize(index) function pointer
  },
  {
    "landmarks_world",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, landmarks_world),  // bytes offset in struct
    nullptr,  // default value
    size_function__HandLandmarks__landmarks_world,  // size() function pointer
    get_const_function__HandLandmarks__landmarks_world,  // get_const(index) function pointer
    get_function__HandLandmarks__landmarks_world,  // get(index) function pointer
    fetch_function__HandLandmarks__landmarks_world,  // fetch(index, &value) function pointer
    assign_function__HandLandmarks__landmarks_world,  // assign(index, value) function pointer
    resize_function__HandLandmarks__landmarks_world  // resize(index) function pointer
  },
  {
    "handed_index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::HandLandmarks, handed_index),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers HandLandmarks_message_members = {
  "handpose_interfaces::msg",  // message namespace
  "HandLandmarks",  // message name
  10,  // number of fields
  sizeof(handpose_interfaces::msg::HandLandmarks),
  HandLandmarks_message_member_array,  // message members
  HandLandmarks_init_function,  // function to initialize message memory (memory has to be allocated)
  HandLandmarks_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t HandLandmarks_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &HandLandmarks_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace handpose_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<handpose_interfaces::msg::HandLandmarks>()
{
  return &::handpose_interfaces::msg::rosidl_typesupport_introspection_cpp::HandLandmarks_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, handpose_interfaces, msg, HandLandmarks)() {
  return &::handpose_interfaces::msg::rosidl_typesupport_introspection_cpp::HandLandmarks_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
