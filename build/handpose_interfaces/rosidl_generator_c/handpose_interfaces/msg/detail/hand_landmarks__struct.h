// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from handpose_interfaces:msg/HandLandmarks.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__STRUCT_H_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__STRUCT_H_

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
// Member 'label'
#include "rosidl_runtime_c/string.h"
// Member 'landmarks_norm'
// Member 'landmarks_canon'
// Member 'landmarks_world'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/HandLandmarks in the package handpose_interfaces.
typedef struct handpose_interfaces__msg__HandLandmarks
{
  std_msgs__msg__Header header;
  /// 같은 프레임 내에서 손 인덱스
  int8_t id;
  /// "left" | "right"
  rosidl_runtime_c__String label;
  /// handedness score
  float score;
  /// 입력 이미지 W
  uint32_t width;
  /// 입력 이미지 H
  uint32_t height;
  /// 21x3 을 1차원으로 평탄화 (length = 63)
  /// (x,y,z) normalized, z: mediapipe scale
  rosidl_runtime_c__float__Sequence landmarks_norm;
  /// (x,y,z) canonical/pixel; x in [0,W], y in [0,H], z in
  rosidl_runtime_c__float__Sequence landmarks_canon;
  /// (x,y,z) meter, origin=hand geometric center (없으면 빈 배열)
  rosidl_runtime_c__float__Sequence landmarks_world;
  /// 선택: multi-hands 정렬 유지용 인덱스(미사용시 -1)
  int32_t handed_index;
} handpose_interfaces__msg__HandLandmarks;

// Struct for a sequence of handpose_interfaces__msg__HandLandmarks.
typedef struct handpose_interfaces__msg__HandLandmarks__Sequence
{
  handpose_interfaces__msg__HandLandmarks * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} handpose_interfaces__msg__HandLandmarks__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__STRUCT_H_
