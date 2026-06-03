// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from fake_tag_interfaces:msg/TagDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fake_tag_interfaces/msg/tag_detection.h"


#ifndef FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__STRUCT_H_
#define FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/TagDetection in the package fake_tag_interfaces.
typedef struct fake_tag_interfaces__msg__TagDetection
{
  int32_t tag_id;
  geometry_msgs__msg__Pose pose;
} fake_tag_interfaces__msg__TagDetection;

// Struct for a sequence of fake_tag_interfaces__msg__TagDetection.
typedef struct fake_tag_interfaces__msg__TagDetection__Sequence
{
  fake_tag_interfaces__msg__TagDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fake_tag_interfaces__msg__TagDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__STRUCT_H_
