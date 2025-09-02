// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from handpose_interfaces:msg/Hands.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "handpose_interfaces/msg/detail/hands__struct.hpp"
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

void Hands_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) handpose_interfaces::msg::Hands(_init);
}

void Hands_fini_function(void * message_memory)
{
  auto typed_message = static_cast<handpose_interfaces::msg::Hands *>(message_memory);
  typed_message->~Hands();
}

size_t size_function__Hands__hands(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<handpose_interfaces::msg::HandLandmarks> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Hands__hands(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<handpose_interfaces::msg::HandLandmarks> *>(untyped_member);
  return &member[index];
}

void * get_function__Hands__hands(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<handpose_interfaces::msg::HandLandmarks> *>(untyped_member);
  return &member[index];
}

void fetch_function__Hands__hands(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const handpose_interfaces::msg::HandLandmarks *>(
    get_const_function__Hands__hands(untyped_member, index));
  auto & value = *reinterpret_cast<handpose_interfaces::msg::HandLandmarks *>(untyped_value);
  value = item;
}

void assign_function__Hands__hands(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<handpose_interfaces::msg::HandLandmarks *>(
    get_function__Hands__hands(untyped_member, index));
  const auto & value = *reinterpret_cast<const handpose_interfaces::msg::HandLandmarks *>(untyped_value);
  item = value;
}

void resize_function__Hands__hands(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<handpose_interfaces::msg::HandLandmarks> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Hands_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::Hands, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "hands",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<handpose_interfaces::msg::HandLandmarks>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces::msg::Hands, hands),  // bytes offset in struct
    nullptr,  // default value
    size_function__Hands__hands,  // size() function pointer
    get_const_function__Hands__hands,  // get_const(index) function pointer
    get_function__Hands__hands,  // get(index) function pointer
    fetch_function__Hands__hands,  // fetch(index, &value) function pointer
    assign_function__Hands__hands,  // assign(index, value) function pointer
    resize_function__Hands__hands  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Hands_message_members = {
  "handpose_interfaces::msg",  // message namespace
  "Hands",  // message name
  2,  // number of fields
  sizeof(handpose_interfaces::msg::Hands),
  Hands_message_member_array,  // message members
  Hands_init_function,  // function to initialize message memory (memory has to be allocated)
  Hands_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Hands_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Hands_message_members,
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
get_message_type_support_handle<handpose_interfaces::msg::Hands>()
{
  return &::handpose_interfaces::msg::rosidl_typesupport_introspection_cpp::Hands_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, handpose_interfaces, msg, Hands)() {
  return &::handpose_interfaces::msg::rosidl_typesupport_introspection_cpp::Hands_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
