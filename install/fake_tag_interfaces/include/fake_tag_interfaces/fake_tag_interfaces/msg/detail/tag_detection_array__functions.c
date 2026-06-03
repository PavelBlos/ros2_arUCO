// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from fake_tag_interfaces:msg/TagDetectionArray.idl
// generated code does not contain a copyright notice
#include "fake_tag_interfaces/msg/detail/tag_detection_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `detections`
#include "fake_tag_interfaces/msg/detail/tag_detection__functions.h"

bool
fake_tag_interfaces__msg__TagDetectionArray__init(fake_tag_interfaces__msg__TagDetectionArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    fake_tag_interfaces__msg__TagDetectionArray__fini(msg);
    return false;
  }
  // detections
  if (!fake_tag_interfaces__msg__TagDetection__Sequence__init(&msg->detections, 0)) {
    fake_tag_interfaces__msg__TagDetectionArray__fini(msg);
    return false;
  }
  return true;
}

void
fake_tag_interfaces__msg__TagDetectionArray__fini(fake_tag_interfaces__msg__TagDetectionArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detections
  fake_tag_interfaces__msg__TagDetection__Sequence__fini(&msg->detections);
}

bool
fake_tag_interfaces__msg__TagDetectionArray__are_equal(const fake_tag_interfaces__msg__TagDetectionArray * lhs, const fake_tag_interfaces__msg__TagDetectionArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // detections
  if (!fake_tag_interfaces__msg__TagDetection__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
fake_tag_interfaces__msg__TagDetectionArray__copy(
  const fake_tag_interfaces__msg__TagDetectionArray * input,
  fake_tag_interfaces__msg__TagDetectionArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // detections
  if (!fake_tag_interfaces__msg__TagDetection__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

fake_tag_interfaces__msg__TagDetectionArray *
fake_tag_interfaces__msg__TagDetectionArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fake_tag_interfaces__msg__TagDetectionArray * msg = (fake_tag_interfaces__msg__TagDetectionArray *)allocator.allocate(sizeof(fake_tag_interfaces__msg__TagDetectionArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(fake_tag_interfaces__msg__TagDetectionArray));
  bool success = fake_tag_interfaces__msg__TagDetectionArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
fake_tag_interfaces__msg__TagDetectionArray__destroy(fake_tag_interfaces__msg__TagDetectionArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    fake_tag_interfaces__msg__TagDetectionArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
fake_tag_interfaces__msg__TagDetectionArray__Sequence__init(fake_tag_interfaces__msg__TagDetectionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fake_tag_interfaces__msg__TagDetectionArray * data = NULL;

  if (size) {
    data = (fake_tag_interfaces__msg__TagDetectionArray *)allocator.zero_allocate(size, sizeof(fake_tag_interfaces__msg__TagDetectionArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = fake_tag_interfaces__msg__TagDetectionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        fake_tag_interfaces__msg__TagDetectionArray__fini(&data[i - 1]);
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
fake_tag_interfaces__msg__TagDetectionArray__Sequence__fini(fake_tag_interfaces__msg__TagDetectionArray__Sequence * array)
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
      fake_tag_interfaces__msg__TagDetectionArray__fini(&array->data[i]);
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

fake_tag_interfaces__msg__TagDetectionArray__Sequence *
fake_tag_interfaces__msg__TagDetectionArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fake_tag_interfaces__msg__TagDetectionArray__Sequence * array = (fake_tag_interfaces__msg__TagDetectionArray__Sequence *)allocator.allocate(sizeof(fake_tag_interfaces__msg__TagDetectionArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = fake_tag_interfaces__msg__TagDetectionArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
fake_tag_interfaces__msg__TagDetectionArray__Sequence__destroy(fake_tag_interfaces__msg__TagDetectionArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    fake_tag_interfaces__msg__TagDetectionArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
fake_tag_interfaces__msg__TagDetectionArray__Sequence__are_equal(const fake_tag_interfaces__msg__TagDetectionArray__Sequence * lhs, const fake_tag_interfaces__msg__TagDetectionArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!fake_tag_interfaces__msg__TagDetectionArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
fake_tag_interfaces__msg__TagDetectionArray__Sequence__copy(
  const fake_tag_interfaces__msg__TagDetectionArray__Sequence * input,
  fake_tag_interfaces__msg__TagDetectionArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(fake_tag_interfaces__msg__TagDetectionArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    fake_tag_interfaces__msg__TagDetectionArray * data =
      (fake_tag_interfaces__msg__TagDetectionArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!fake_tag_interfaces__msg__TagDetectionArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          fake_tag_interfaces__msg__TagDetectionArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!fake_tag_interfaces__msg__TagDetectionArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
