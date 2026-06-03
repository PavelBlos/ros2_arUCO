// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from fake_tag_interfaces:msg/TagDetectionArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "fake_tag_interfaces/msg/detail/tag_detection_array__rosidl_typesupport_introspection_c.h"
#include "fake_tag_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "fake_tag_interfaces/msg/detail/tag_detection_array__functions.h"
#include "fake_tag_interfaces/msg/detail/tag_detection_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `detections`
#include "fake_tag_interfaces/msg/tag_detection.h"
// Member `detections`
#include "fake_tag_interfaces/msg/detail/tag_detection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  fake_tag_interfaces__msg__TagDetectionArray__init(message_memory);
}

void fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_fini_function(void * message_memory)
{
  fake_tag_interfaces__msg__TagDetectionArray__fini(message_memory);
}

size_t fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__size_function__TagDetectionArray__detections(
  const void * untyped_member)
{
  const fake_tag_interfaces__msg__TagDetection__Sequence * member =
    (const fake_tag_interfaces__msg__TagDetection__Sequence *)(untyped_member);
  return member->size;
}

const void * fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__get_const_function__TagDetectionArray__detections(
  const void * untyped_member, size_t index)
{
  const fake_tag_interfaces__msg__TagDetection__Sequence * member =
    (const fake_tag_interfaces__msg__TagDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__get_function__TagDetectionArray__detections(
  void * untyped_member, size_t index)
{
  fake_tag_interfaces__msg__TagDetection__Sequence * member =
    (fake_tag_interfaces__msg__TagDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__fetch_function__TagDetectionArray__detections(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const fake_tag_interfaces__msg__TagDetection * item =
    ((const fake_tag_interfaces__msg__TagDetection *)
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__get_const_function__TagDetectionArray__detections(untyped_member, index));
  fake_tag_interfaces__msg__TagDetection * value =
    (fake_tag_interfaces__msg__TagDetection *)(untyped_value);
  *value = *item;
}

void fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__assign_function__TagDetectionArray__detections(
  void * untyped_member, size_t index, const void * untyped_value)
{
  fake_tag_interfaces__msg__TagDetection * item =
    ((fake_tag_interfaces__msg__TagDetection *)
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__get_function__TagDetectionArray__detections(untyped_member, index));
  const fake_tag_interfaces__msg__TagDetection * value =
    (const fake_tag_interfaces__msg__TagDetection *)(untyped_value);
  *item = *value;
}

bool fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__resize_function__TagDetectionArray__detections(
  void * untyped_member, size_t size)
{
  fake_tag_interfaces__msg__TagDetection__Sequence * member =
    (fake_tag_interfaces__msg__TagDetection__Sequence *)(untyped_member);
  fake_tag_interfaces__msg__TagDetection__Sequence__fini(member);
  return fake_tag_interfaces__msg__TagDetection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fake_tag_interfaces__msg__TagDetectionArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detections",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fake_tag_interfaces__msg__TagDetectionArray, detections),  // bytes offset in struct
    NULL,  // default value
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__size_function__TagDetectionArray__detections,  // size() function pointer
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__get_const_function__TagDetectionArray__detections,  // get_const(index) function pointer
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__get_function__TagDetectionArray__detections,  // get(index) function pointer
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__fetch_function__TagDetectionArray__detections,  // fetch(index, &value) function pointer
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__assign_function__TagDetectionArray__detections,  // assign(index, value) function pointer
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__resize_function__TagDetectionArray__detections  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_members = {
  "fake_tag_interfaces__msg",  // message namespace
  "TagDetectionArray",  // message name
  2,  // number of fields
  sizeof(fake_tag_interfaces__msg__TagDetectionArray),
  false,  // has_any_key_member_
  fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_member_array,  // message members
  fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_type_support_handle = {
  0,
  &fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_members,
  get_message_typesupport_handle_function,
  &fake_tag_interfaces__msg__TagDetectionArray__get_type_hash,
  &fake_tag_interfaces__msg__TagDetectionArray__get_type_description,
  &fake_tag_interfaces__msg__TagDetectionArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_fake_tag_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fake_tag_interfaces, msg, TagDetectionArray)() {
  fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fake_tag_interfaces, msg, TagDetection)();
  if (!fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_type_support_handle.typesupport_identifier) {
    fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &fake_tag_interfaces__msg__TagDetectionArray__rosidl_typesupport_introspection_c__TagDetectionArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
