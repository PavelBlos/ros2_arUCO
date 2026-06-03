// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from fake_tag_interfaces:msg/TagDetection.idl
// generated code does not contain a copyright notice
#ifndef FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "fake_tag_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "fake_tag_interfaces/msg/detail/tag_detection__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fake_tag_interfaces
bool cdr_serialize_fake_tag_interfaces__msg__TagDetection(
  const fake_tag_interfaces__msg__TagDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fake_tag_interfaces
bool cdr_deserialize_fake_tag_interfaces__msg__TagDetection(
  eprosima::fastcdr::Cdr &,
  fake_tag_interfaces__msg__TagDetection * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fake_tag_interfaces
size_t get_serialized_size_fake_tag_interfaces__msg__TagDetection(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fake_tag_interfaces
size_t max_serialized_size_fake_tag_interfaces__msg__TagDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fake_tag_interfaces
bool cdr_serialize_key_fake_tag_interfaces__msg__TagDetection(
  const fake_tag_interfaces__msg__TagDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fake_tag_interfaces
size_t get_serialized_size_key_fake_tag_interfaces__msg__TagDetection(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fake_tag_interfaces
size_t max_serialized_size_key_fake_tag_interfaces__msg__TagDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fake_tag_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, fake_tag_interfaces, msg, TagDetection)();

#ifdef __cplusplus
}
#endif

#endif  // FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
