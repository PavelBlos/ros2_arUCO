// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from fake_tag_interfaces:msg/TagDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fake_tag_interfaces/msg/tag_detection.hpp"


#ifndef FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__STRUCT_HPP_
#define FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__fake_tag_interfaces__msg__TagDetection __attribute__((deprecated))
#else
# define DEPRECATED__fake_tag_interfaces__msg__TagDetection __declspec(deprecated)
#endif

namespace fake_tag_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TagDetection_
{
  using Type = TagDetection_<ContainerAllocator>;

  explicit TagDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tag_id = 0l;
    }
  }

  explicit TagDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tag_id = 0l;
    }
  }

  // field types and members
  using _tag_id_type =
    int32_t;
  _tag_id_type tag_id;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__tag_id(
    const int32_t & _arg)
  {
    this->tag_id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fake_tag_interfaces::msg::TagDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const fake_tag_interfaces::msg::TagDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fake_tag_interfaces::msg::TagDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fake_tag_interfaces::msg::TagDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fake_tag_interfaces::msg::TagDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fake_tag_interfaces::msg::TagDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fake_tag_interfaces::msg::TagDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fake_tag_interfaces::msg::TagDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fake_tag_interfaces::msg::TagDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fake_tag_interfaces::msg::TagDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fake_tag_interfaces__msg__TagDetection
    std::shared_ptr<fake_tag_interfaces::msg::TagDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fake_tag_interfaces__msg__TagDetection
    std::shared_ptr<fake_tag_interfaces::msg::TagDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TagDetection_ & other) const
  {
    if (this->tag_id != other.tag_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const TagDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TagDetection_

// alias to use template instance with default allocator
using TagDetection =
  fake_tag_interfaces::msg::TagDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace fake_tag_interfaces

#endif  // FAKE_TAG_INTERFACES__MSG__DETAIL__TAG_DETECTION__STRUCT_HPP_
