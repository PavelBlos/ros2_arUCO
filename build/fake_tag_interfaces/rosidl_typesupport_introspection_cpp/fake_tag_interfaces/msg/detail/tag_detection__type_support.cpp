// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from fake_tag_interfaces:msg/TagDetection.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "fake_tag_interfaces/msg/detail/tag_detection__functions.h"
#include "fake_tag_interfaces/msg/detail/tag_detection__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace fake_tag_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TagDetection_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) fake_tag_interfaces::msg::TagDetection(_init);
}

void TagDetection_fini_function(void * message_memory)
{
  auto typed_message = static_cast<fake_tag_interfaces::msg::TagDetection *>(message_memory);
  typed_message->~TagDetection();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TagDetection_message_member_array[2] = {
  {
    "tag_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fake_tag_interfaces::msg::TagDetection, tag_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fake_tag_interfaces::msg::TagDetection, pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TagDetection_message_members = {
  "fake_tag_interfaces::msg",  // message namespace
  "TagDetection",  // message name
  2,  // number of fields
  sizeof(fake_tag_interfaces::msg::TagDetection),
  false,  // has_any_key_member_
  TagDetection_message_member_array,  // message members
  TagDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  TagDetection_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TagDetection_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TagDetection_message_members,
  get_message_typesupport_handle_function,
  &fake_tag_interfaces__msg__TagDetection__get_type_hash,
  &fake_tag_interfaces__msg__TagDetection__get_type_description,
  &fake_tag_interfaces__msg__TagDetection__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace fake_tag_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<fake_tag_interfaces::msg::TagDetection>()
{
  return &::fake_tag_interfaces::msg::rosidl_typesupport_introspection_cpp::TagDetection_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, fake_tag_interfaces, msg, TagDetection)() {
  return &::fake_tag_interfaces::msg::rosidl_typesupport_introspection_cpp::TagDetection_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
