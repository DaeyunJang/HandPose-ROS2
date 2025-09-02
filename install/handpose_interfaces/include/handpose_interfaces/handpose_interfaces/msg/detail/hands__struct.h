// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from handpose_interfaces:msg/Hands.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__STRUCT_H_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'hands'
#include "handpose_interfaces/msg/detail/hand_landmarks__struct.h"

/// Struct defined in msg/Hands in the package handpose_interfaces.
typedef struct handpose_interfaces__msg__Hands
{
  std_msgs__msg__Header header;
  handpose_interfaces__msg__HandLandmarks__Sequence hands;
} handpose_interfaces__msg__Hands;

// Struct for a sequence of handpose_interfaces__msg__Hands.
typedef struct handpose_interfaces__msg__Hands__Sequence
{
  handpose_interfaces__msg__Hands * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} handpose_interfaces__msg__Hands__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HANDS__STRUCT_H_
