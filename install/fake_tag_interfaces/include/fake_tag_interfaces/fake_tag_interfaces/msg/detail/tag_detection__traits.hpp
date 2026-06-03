// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fake_tag_interfaces:msg/TagDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fake_tag_interfaces/msg/tag_detection.hpp"


#ifndef FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__TRAITS_HPP_
#define FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "fake_tag_interfaces/msg/detail/tag_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace fake_tag_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TagDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: tag_id
  {
    out << "tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_id, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TagDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.tag_id, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TagDetection & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace fake_tag_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use fake_tag_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const fake_tag_interfaces::msg::TagDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  fake_tag_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fake_tag_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const fake_tag_interfaces::msg::TagDetection & msg)
{
  return fake_tag_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<fake_tag_interfaces::msg::TagDetection>()
{
  return "fake_tag_interfaces::msg::TagDetection";
}

template<>
inline const char * name<fake_tag_interfaces::msg::TagDetection>()
{
  return "fake_tag_interfaces/msg/TagDetection";
}

template<>
struct has_fixed_size<fake_tag_interfaces::msg::TagDetection>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<fake_tag_interfaces::msg::TagDetection>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<fake_tag_interfaces::msg::TagDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__TRAITS_HPP_
