// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from seal_msgs:msg/SealData.idl
// generated code does not contain a copyright notice
#include "seal_msgs/msg/detail/seal_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
seal_msgs__msg__SealData__init(seal_msgs__msg__SealData * msg)
{
  if (!msg) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    seal_msgs__msg__SealData__fini(msg);
    return false;
  }
  return true;
}

void
seal_msgs__msg__SealData__fini(seal_msgs__msg__SealData * msg)
{
  if (!msg) {
    return;
  }
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
}

bool
seal_msgs__msg__SealData__are_equal(const seal_msgs__msg__SealData * lhs, const seal_msgs__msg__SealData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
seal_msgs__msg__SealData__copy(
  const seal_msgs__msg__SealData * input,
  seal_msgs__msg__SealData * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

seal_msgs__msg__SealData *
seal_msgs__msg__SealData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__msg__SealData * msg = (seal_msgs__msg__SealData *)allocator.allocate(sizeof(seal_msgs__msg__SealData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(seal_msgs__msg__SealData));
  bool success = seal_msgs__msg__SealData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
seal_msgs__msg__SealData__destroy(seal_msgs__msg__SealData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    seal_msgs__msg__SealData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
seal_msgs__msg__SealData__Sequence__init(seal_msgs__msg__SealData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__msg__SealData * data = NULL;

  if (size) {
    data = (seal_msgs__msg__SealData *)allocator.zero_allocate(size, sizeof(seal_msgs__msg__SealData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = seal_msgs__msg__SealData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        seal_msgs__msg__SealData__fini(&data[i - 1]);
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
seal_msgs__msg__SealData__Sequence__fini(seal_msgs__msg__SealData__Sequence * array)
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
      seal_msgs__msg__SealData__fini(&array->data[i]);
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

seal_msgs__msg__SealData__Sequence *
seal_msgs__msg__SealData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__msg__SealData__Sequence * array = (seal_msgs__msg__SealData__Sequence *)allocator.allocate(sizeof(seal_msgs__msg__SealData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = seal_msgs__msg__SealData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
seal_msgs__msg__SealData__Sequence__destroy(seal_msgs__msg__SealData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    seal_msgs__msg__SealData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
seal_msgs__msg__SealData__Sequence__are_equal(const seal_msgs__msg__SealData__Sequence * lhs, const seal_msgs__msg__SealData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!seal_msgs__msg__SealData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
seal_msgs__msg__SealData__Sequence__copy(
  const seal_msgs__msg__SealData__Sequence * input,
  seal_msgs__msg__SealData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(seal_msgs__msg__SealData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    seal_msgs__msg__SealData * data =
      (seal_msgs__msg__SealData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!seal_msgs__msg__SealData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          seal_msgs__msg__SealData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!seal_msgs__msg__SealData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
