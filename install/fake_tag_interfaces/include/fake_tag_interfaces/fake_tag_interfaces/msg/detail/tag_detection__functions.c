// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from fake_tag_interfaces:msg/TagDetection.idl
// generated code does not contain a copyright notice
#include "fake_tag_interfaces/msg/detail/tag_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
fake_tag_interfaces__msg__TagDetection__init(fake_tag_interfaces__msg__TagDetection * msg)
{
  if (!msg) {
    return false;
  }
  // tag_id
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    fake_tag_interfaces__msg__TagDetection__fini(msg);
    return false;
  }
  return true;
}

void
fake_tag_interfaces__msg__TagDetection__fini(fake_tag_interfaces__msg__TagDetection * msg)
{
  if (!msg) {
    return;
  }
  // tag_id
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
fake_tag_interfaces__msg__TagDetection__are_equal(const fake_tag_interfaces__msg__TagDetection * lhs, const fake_tag_interfaces__msg__TagDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tag_id
  if (lhs->tag_id != rhs->tag_id) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
fake_tag_interfaces__msg__TagDetection__copy(
  const fake_tag_interfaces__msg__TagDetection * input,
  fake_tag_interfaces__msg__TagDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // tag_id
  output->tag_id = input->tag_id;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

fake_tag_interfaces__msg__TagDetection *
fake_tag_interfaces__msg__TagDetection__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fake_tag_interfaces__msg__TagDetection * msg = (fake_tag_interfaces__msg__TagDetection *)allocator.allocate(sizeof(fake_tag_interfaces__msg__TagDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(fake_tag_interfaces__msg__TagDetection));
  bool success = fake_tag_interfaces__msg__TagDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
fake_tag_interfaces__msg__TagDetection__destroy(fake_tag_interfaces__msg__TagDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    fake_tag_interfaces__msg__TagDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
fake_tag_interfaces__msg__TagDetection__Sequence__init(fake_tag_interfaces__msg__TagDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fake_tag_interfaces__msg__TagDetection * data = NULL;

  if (size) {
    data = (fake_tag_interfaces__msg__TagDetection *)allocator.zero_allocate(size, sizeof(fake_tag_interfaces__msg__TagDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = fake_tag_interfaces__msg__TagDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        fake_tag_interfaces__msg__TagDetection__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
fake_tag_interfaces__msg__TagDetection__Sequence__fini(fake_tag_interfaces__msg__TagDetection__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      fake_tag_interfaces__msg__TagDetection__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

fake_tag_interfaces__msg__TagDetection__Sequence *
fake_tag_interfaces__msg__TagDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fake_tag_interfaces__msg__TagDetection__Sequence * array = (fake_tag_interfaces__msg__TagDetection__Sequence *)allocator.allocate(sizeof(fake_tag_interfaces__msg__TagDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = fake_tag_interfaces__msg__TagDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
fake_tag_interfaces__msg__TagDetection__Sequence__destroy(fake_tag_interfaces__msg__TagDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    fake_tag_interfaces__msg__TagDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
fake_tag_interfaces__msg__TagDetection__Sequence__are_equal(const fake_tag_interfaces__msg__TagDetection__Sequence * lhs, const fake_tag_interfaces__msg__TagDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!fake_tag_interfaces__msg__TagDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
fake_tag_interfaces__msg__TagDetection__Sequence__copy(
  const fake_tag_interfaces__msg__TagDetection__Sequence * input,
  fake_tag_interfaces__msg__TagDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(fake_tag_interfaces__msg__TagDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    fake_tag_interfaces__msg__TagDetection * data =
      (fake_tag_interfaces__msg__TagDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!fake_tag_interfaces__msg__TagDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          fake_tag_interfaces__msg__TagDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!fake_tag_interfaces__msg__TagDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
