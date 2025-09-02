// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from handpose_interfaces:msg/HandLandmarks.idl
// generated code does not contain a copyright notice

#ifndef HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__FUNCTIONS_H_
#define HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "handpose_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "handpose_interfaces/msg/detail/hand_landmarks__struct.h"

/// Initialize msg/HandLandmarks message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * handpose_interfaces__msg__HandLandmarks
 * )) before or use
 * handpose_interfaces__msg__HandLandmarks__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
bool
handpose_interfaces__msg__HandLandmarks__init(handpose_interfaces__msg__HandLandmarks * msg);

/// Finalize msg/HandLandmarks message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
void
handpose_interfaces__msg__HandLandmarks__fini(handpose_interfaces__msg__HandLandmarks * msg);

/// Create msg/HandLandmarks message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * handpose_interfaces__msg__HandLandmarks__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
handpose_interfaces__msg__HandLandmarks *
handpose_interfaces__msg__HandLandmarks__create();

/// Destroy msg/HandLandmarks message.
/**
 * It calls
 * handpose_interfaces__msg__HandLandmarks__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
void
handpose_interfaces__msg__HandLandmarks__destroy(handpose_interfaces__msg__HandLandmarks * msg);

/// Check for msg/HandLandmarks message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
bool
handpose_interfaces__msg__HandLandmarks__are_equal(const handpose_interfaces__msg__HandLandmarks * lhs, const handpose_interfaces__msg__HandLandmarks * rhs);

/// Copy a msg/HandLandmarks message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
bool
handpose_interfaces__msg__HandLandmarks__copy(
  const handpose_interfaces__msg__HandLandmarks * input,
  handpose_interfaces__msg__HandLandmarks * output);

/// Initialize array of msg/HandLandmarks messages.
/**
 * It allocates the memory for the number of elements and calls
 * handpose_interfaces__msg__HandLandmarks__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
bool
handpose_interfaces__msg__HandLandmarks__Sequence__init(handpose_interfaces__msg__HandLandmarks__Sequence * array, size_t size);

/// Finalize array of msg/HandLandmarks messages.
/**
 * It calls
 * handpose_interfaces__msg__HandLandmarks__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
void
handpose_interfaces__msg__HandLandmarks__Sequence__fini(handpose_interfaces__msg__HandLandmarks__Sequence * array);

/// Create array of msg/HandLandmarks messages.
/**
 * It allocates the memory for the array and calls
 * handpose_interfaces__msg__HandLandmarks__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
handpose_interfaces__msg__HandLandmarks__Sequence *
handpose_interfaces__msg__HandLandmarks__Sequence__create(size_t size);

/// Destroy array of msg/HandLandmarks messages.
/**
 * It calls
 * handpose_interfaces__msg__HandLandmarks__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
void
handpose_interfaces__msg__HandLandmarks__Sequence__destroy(handpose_interfaces__msg__HandLandmarks__Sequence * array);

/// Check for msg/HandLandmarks message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
bool
handpose_interfaces__msg__HandLandmarks__Sequence__are_equal(const handpose_interfaces__msg__HandLandmarks__Sequence * lhs, const handpose_interfaces__msg__HandLandmarks__Sequence * rhs);

/// Copy an array of msg/HandLandmarks messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_handpose_interfaces
bool
handpose_interfaces__msg__HandLandmarks__Sequence__copy(
  const handpose_interfaces__msg__HandLandmarks__Sequence * input,
  handpose_interfaces__msg__HandLandmarks__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HANDPOSE_INTERFACES__MSG__DETAIL__HAND_LANDMARKS__FUNCTIONS_H_
