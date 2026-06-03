// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from fake_tag_interfaces:msg/TagDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fake_tag_interfaces/msg/tag_detection_array.h"


#ifndef FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION_ARRAY__STRUCT_H_
#define FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION_ARRAY__STRUCT_H_

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
// Member 'detections'
#include "fake_tag_interfaces/msg/detail/tag_detection__struct.h"

/// Struct defined in msg/TagDetectionArray in the package fake_tag_interfaces.
typedef struct fake_tag_interfaces__msg__TagDetectionArray
{
  std_msgs__msg__Header header;
  fake_tag_interfaces__msg__TagDetection__Sequence detections;
} fake_tag_interfaces__msg__TagDetectionArray;

// Struct for a sequence of fake_tag_interfaces__msg__TagDetectionArray.
typedef struct fake_tag_interfaces__msg__TagDetectionArray__Sequence
{
  fake_tag_interfaces__msg__TagDetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fake_tag_interfaces__msg__TagDetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION_ARRAY__STRUCT_H_
