// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from handpose_interfaces:msg/HandLandmarks.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "handpose_interfaces/msg/detail/hand_landmarks__rosidl_typesupport_introspection_c.h"
#include "handpose_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "handpose_interfaces/msg/detail/hand_landmarks__functions.h"
#include "handpose_interfaces/msg/detail/hand_landmarks__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `label`
#include "rosidl_runtime_c/string_functions.h"
// Member `landmarks_norm`
// Member `landmarks_canon`
// Member `landmarks_world`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  handpose_interfaces__msg__HandLandmarks__init(message_memory);
}

void handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_fini_function(void * message_memory)
{
  handpose_interfaces__msg__HandLandmarks__fini(message_memory);
}

size_t handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__size_function__HandLandmarks__landmarks_norm(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_norm(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_norm(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__fetch_function__HandLandmarks__landmarks_norm(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_norm(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__assign_function__HandLandmarks__landmarks_norm(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_norm(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__resize_function__HandLandmarks__landmarks_norm(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__size_function__HandLandmarks__landmarks_canon(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_canon(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_canon(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__fetch_function__HandLandmarks__landmarks_canon(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_canon(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__assign_function__HandLandmarks__landmarks_canon(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_canon(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__resize_function__HandLandmarks__landmarks_canon(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__size_function__HandLandmarks__landmarks_world(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_world(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_world(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__fetch_function__HandLandmarks__landmarks_world(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_world(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__assign_function__HandLandmarks__landmarks_world(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_world(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__resize_function__HandLandmarks__landmarks_world(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_member_array[10] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "label",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, label),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "score",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, score),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "landmarks_norm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, landmarks_norm),  // bytes offset in struct
    NULL,  // default value
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__size_function__HandLandmarks__landmarks_norm,  // size() function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_norm,  // get_const(index) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_norm,  // get(index) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__fetch_function__HandLandmarks__landmarks_norm,  // fetch(index, &value) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__assign_function__HandLandmarks__landmarks_norm,  // assign(index, value) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__resize_function__HandLandmarks__landmarks_norm  // resize(index) function pointer
  },
  {
    "landmarks_canon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, landmarks_canon),  // bytes offset in struct
    NULL,  // default value
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__size_function__HandLandmarks__landmarks_canon,  // size() function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_canon,  // get_const(index) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_canon,  // get(index) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__fetch_function__HandLandmarks__landmarks_canon,  // fetch(index, &value) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__assign_function__HandLandmarks__landmarks_canon,  // assign(index, value) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__resize_function__HandLandmarks__landmarks_canon  // resize(index) function pointer
  },
  {
    "landmarks_world",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, landmarks_world),  // bytes offset in struct
    NULL,  // default value
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__size_function__HandLandmarks__landmarks_world,  // size() function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_const_function__HandLandmarks__landmarks_world,  // get_const(index) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__get_function__HandLandmarks__landmarks_world,  // get(index) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__fetch_function__HandLandmarks__landmarks_world,  // fetch(index, &value) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__assign_function__HandLandmarks__landmarks_world,  // assign(index, value) function pointer
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__resize_function__HandLandmarks__landmarks_world  // resize(index) function pointer
  },
  {
    "handed_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(handpose_interfaces__msg__HandLandmarks, handed_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_members = {
  "handpose_interfaces__msg",  // message namespace
  "HandLandmarks",  // message name
  10,  // number of fields
  sizeof(handpose_interfaces__msg__HandLandmarks),
  handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_member_array,  // message members
  handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_init_function,  // function to initialize message memory (memory has to be allocated)
  handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_type_support_handle = {
  0,
  &handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_handpose_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, handpose_interfaces, msg, HandLandmarks)() {
  handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_type_support_handle.typesupport_identifier) {
    handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &handpose_interfaces__msg__HandLandmarks__rosidl_typesupport_introspection_c__HandLandmarks_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
