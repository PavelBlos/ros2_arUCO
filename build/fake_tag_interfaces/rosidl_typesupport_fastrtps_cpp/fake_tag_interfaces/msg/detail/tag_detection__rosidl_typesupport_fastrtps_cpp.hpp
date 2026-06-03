// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from fake_tag_interfaces:msg/TagDetection.idl
// generated code does not contain a copyright notice

#ifndef FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "fake_tag_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "fake_tag_interfaces/msg/detail/tag_detection__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace fake_tag_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fake_tag_interfaces
cdr_serialize(
  const fake_tag_interfaces::msg::TagDetection & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fake_tag_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  fake_tag_interfaces::msg::TagDetection & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fake_tag_interfaces
get_serialized_size(
  const fake_tag_interfaces::msg::TagDetection & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fake_tag_interfaces
max_serialized_size_TagDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fake_tag_interfaces
cdr_serialize_key(
  const fake_tag_interfaces::msg::TagDetection & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fake_tag_interfaces
get_serialized_size_key(
  const fake_tag_interfaces::msg::TagDetection & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fake_tag_interfaces
max_serialized_size_key_TagDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace fake_tag_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fake_tag_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, fake_tag_interfaces, msg, TagDetection)();

#ifdef __cplusplus
}
#endif

#endif  // FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
