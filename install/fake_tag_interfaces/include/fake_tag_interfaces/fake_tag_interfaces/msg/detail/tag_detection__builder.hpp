// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from fake_tag_interfaces:msg/TagDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fake_tag_interfaces/msg/tag_detection.hpp"


#ifndef FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__BUILDER_HPP_
#define FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "fake_tag_interfaces/msg/detail/tag_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace fake_tag_interfaces
{

namespace msg
{

namespace builder
{

class Init_TagDetection_pose
{
public:
  explicit Init_TagDetection_pose(::fake_tag_interfaces::msg::TagDetection & msg)
  : msg_(msg)
  {}
  ::fake_tag_interfaces::msg::TagDetection pose(::fake_tag_interfaces::msg::TagDetection::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fake_tag_interfaces::msg::TagDetection msg_;
};

class Init_TagDetection_tag_id
{
public:
  Init_TagDetection_tag_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TagDetection_pose tag_id(::fake_tag_interfaces::msg::TagDetection::_tag_id_type arg)
  {
    msg_.tag_id = std::move(arg);
    return Init_TagDetection_pose(msg_);
  }

private:
  ::fake_tag_interfaces::msg::TagDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::fake_tag_interfaces::msg::TagDetection>()
{
  return fake_tag_interfaces::msg::builder::Init_TagDetection_tag_id();
}

}  // namespace fake_tag_interfaces

#endif  // FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__BUILDER_HPP_
