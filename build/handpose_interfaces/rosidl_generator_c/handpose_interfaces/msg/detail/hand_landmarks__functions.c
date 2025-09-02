// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from handpose_interfaces:msg/HandLandmarks.idl
// generated code does not contain a copyright notice
#include "handpose_interfaces/msg/detail/hand_landmarks__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `label`
#include "rosidl_runtime_c/string_functions.h"
// Member `landmarks_norm`
// Member `landmarks_canon`
// Member `landmarks_world`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
handpose_interfaces__msg__HandLandmarks__init(handpose_interfaces__msg__HandLandmarks * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    handpose_interfaces__msg__HandLandmarks__fini(msg);
    return false;
  }
  // id
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    handpose_interfaces__msg__HandLandmarks__fini(msg);
    return false;
  }
  // score
  // width
  // height
  // landmarks_norm
  if (!rosidl_runtime_c__float__Sequence__init(&msg->landmarks_norm, 0)) {
    handpose_interfaces__msg__HandLandmarks__fini(msg);
    return false;
  }
  // landmarks_canon
  if (!rosidl_runtime_c__float__Sequence__init(&msg->landmarks_canon, 0)) {
    handpose_interfaces__msg__HandLandmarks__fini(msg);
    return false;
  }
  // landmarks_world
  if (!rosidl_runtime_c__float__Sequence__init(&msg->landmarks_world, 0)) {
    handpose_interfaces__msg__HandLandmarks__fini(msg);
    return false;
  }
  // handed_index
  return true;
}

void
handpose_interfaces__msg__HandLandmarks__fini(handpose_interfaces__msg__HandLandmarks * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // score
  // width
  // height
  // landmarks_norm
  rosidl_runtime_c__float__Sequence__fini(&msg->landmarks_norm);
  // landmarks_canon
  rosidl_runtime_c__float__Sequence__fini(&msg->landmarks_canon);
  // landmarks_world
  rosidl_runtime_c__float__Sequence__fini(&msg->landmarks_world);
  // handed_index
}

bool
handpose_interfaces__msg__HandLandmarks__are_equal(const handpose_interfaces__msg__HandLandmarks * lhs, const handpose_interfaces__msg__HandLandmarks * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // score
  if (lhs->score != rhs->score) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // landmarks_norm
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->landmarks_norm), &(rhs->landmarks_norm)))
  {
    return false;
  }
  // landmarks_canon
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->landmarks_canon), &(rhs->landmarks_canon)))
  {
    return false;
  }
  // landmarks_world
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->landmarks_world), &(rhs->landmarks_world)))
  {
    return false;
  }
  // handed_index
  if (lhs->handed_index != rhs->handed_index) {
    return false;
  }
  return true;
}

bool
handpose_interfaces__msg__HandLandmarks__copy(
  const handpose_interfaces__msg__HandLandmarks * input,
  handpose_interfaces__msg__HandLandmarks * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // id
  output->id = input->id;
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // score
  output->score = input->score;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // landmarks_norm
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->landmarks_norm), &(output->landmarks_norm)))
  {
    return false;
  }
  // landmarks_canon
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->landmarks_canon), &(output->landmarks_canon)))
  {
    return false;
  }
  // landmarks_world
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->landmarks_world), &(output->landmarks_world)))
  {
    return false;
  }
  // handed_index
  output->handed_index = input->handed_index;
  return true;
}

handpose_interfaces__msg__HandLandmarks *
handpose_interfaces__msg__HandLandmarks__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  handpose_interfaces__msg__HandLandmarks * msg = (handpose_interfaces__msg__HandLandmarks *)allocator.allocate(sizeof(handpose_interfaces__msg__HandLandmarks), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(handpose_interfaces__msg__HandLandmarks));
  bool success = handpose_interfaces__msg__HandLandmarks__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
handpose_interfaces__msg__HandLandmarks__destroy(handpose_interfaces__msg__HandLandmarks * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    handpose_interfaces__msg__HandLandmarks__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
handpose_interfaces__msg__HandLandmarks__Sequence__init(handpose_interfaces__msg__HandLandmarks__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  handpose_interfaces__msg__HandLandmarks * data = NULL;

  if (size) {
    data = (handpose_interfaces__msg__HandLandmarks *)allocator.zero_allocate(size, sizeof(handpose_interfaces__msg__HandLandmarks), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = handpose_interfaces__msg__HandLandmarks__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        handpose_interfaces__msg__HandLandmarks__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
handpose_interfaces__msg__HandLandmarks__Sequence__fini(handpose_interfaces__msg__HandLandmarks__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      handpose_interfaces__msg__HandLandmarks__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

handpose_interfaces__msg__HandLandmarks__Sequence *
handpose_interfaces__msg__HandLandmarks__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  handpose_interfaces__msg__HandLandmarks__Sequence * array = (handpose_interfaces__msg__HandLandmarks__Sequence *)allocator.allocate(sizeof(handpose_interfaces__msg__HandLandmarks__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = handpose_interfaces__msg__HandLandmarks__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
handpose_interfaces__msg__HandLandmarks__Sequence__destroy(handpose_interfaces__msg__HandLandmarks__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    handpose_interfaces__msg__HandLandmarks__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
handpose_interfaces__msg__HandLandmarks__Sequence__are_equal(const handpose_interfaces__msg__HandLandmarks__Sequence * lhs, const handpose_interfaces__msg__HandLandmarks__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!handpose_interfaces__msg__HandLandmarks__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
handpose_interfaces__msg__HandLandmarks__Sequence__copy(
  const handpose_interfaces__msg__HandLandmarks__Sequence * input,
  handpose_interfaces__msg__HandLandmarks__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(handpose_interfaces__msg__HandLandmarks);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    handpose_interfaces__msg__HandLandmarks * data =
      (handpose_interfaces__msg__HandLandmarks *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!handpose_interfaces__msg__HandLandmarks__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          handpose_interfaces__msg__HandLandmarks__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!handpose_interfaces__msg__HandLandmarks__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
