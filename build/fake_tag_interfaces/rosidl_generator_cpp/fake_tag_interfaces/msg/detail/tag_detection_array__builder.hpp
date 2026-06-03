// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from fake_tag_interfaces:msg/TagDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fake_tag_interfaces/msg/tag_detection_array.hpp"


#ifndef FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION_ARRAY__BUILDER_HPP_
#define FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "fake_tag_interfaces/msg/detail/tag_detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace fake_tag_interfaces
{

namespace msg
{

namespace builder
{

class Init_TagDetectionArray_detections
{
public:
  explicit Init_TagDetectionArray_detections(::fake_tag_interfaces::msg::TagDetectionArray & msg)
  : msg_(msg)
  {}
  ::fake_tag_interfaces::msg::TagDetectionArray detections(::fake_tag_interfaces::msg::TagDetectionArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fake_tag_interfaces::msg::TagDetectionArray msg_;
};

class Init_TagDetectionArray_header
{
public:
  Init_TagDetectionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TagDetectionArray_detections header(::fake_tag_interfaces::msg::TagDetectionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TagDetectionArray_detections(msg_);
  }

private:
  ::fake_tag_interfaces::msg::TagDetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::fake_tag_interfaces::msg::TagDetectionArray>()
{
  return fake_tag_interfaces::msg::builder::Init_TagDetectionArray_header();
}

}  // namespace fake_tag_interfaces

#endif  // FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION_ARRAY__BUILDER_HPP_
