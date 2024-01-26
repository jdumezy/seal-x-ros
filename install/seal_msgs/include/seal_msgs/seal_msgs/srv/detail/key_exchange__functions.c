// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from seal_msgs:srv/KeyExchange.idl
// generated code does not contain a copyright notice
#include "seal_msgs/srv/detail/key_exchange__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `public_key`
#include "rosidl_runtime_c/string_functions.h"

bool
seal_msgs__srv__KeyExchange_Request__init(seal_msgs__srv__KeyExchange_Request * msg)
{
  if (!msg) {
    return false;
  }
  // public_key
  if (!rosidl_runtime_c__String__init(&msg->public_key)) {
    seal_msgs__srv__KeyExchange_Request__fini(msg);
    return false;
  }
  return true;
}

void
seal_msgs__srv__KeyExchange_Request__fini(seal_msgs__srv__KeyExchange_Request * msg)
{
  if (!msg) {
    return;
  }
  // public_key
  rosidl_runtime_c__String__fini(&msg->public_key);
}

bool
seal_msgs__srv__KeyExchange_Request__are_equal(const seal_msgs__srv__KeyExchange_Request * lhs, const seal_msgs__srv__KeyExchange_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // public_key
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->public_key), &(rhs->public_key)))
  {
    return false;
  }
  return true;
}

bool
seal_msgs__srv__KeyExchange_Request__copy(
  const seal_msgs__srv__KeyExchange_Request * input,
  seal_msgs__srv__KeyExchange_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // public_key
  if (!rosidl_runtime_c__String__copy(
      &(input->public_key), &(output->public_key)))
  {
    return false;
  }
  return true;
}

seal_msgs__srv__KeyExchange_Request *
seal_msgs__srv__KeyExchange_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__srv__KeyExchange_Request * msg = (seal_msgs__srv__KeyExchange_Request *)allocator.allocate(sizeof(seal_msgs__srv__KeyExchange_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(seal_msgs__srv__KeyExchange_Request));
  bool success = seal_msgs__srv__KeyExchange_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
seal_msgs__srv__KeyExchange_Request__destroy(seal_msgs__srv__KeyExchange_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    seal_msgs__srv__KeyExchange_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
seal_msgs__srv__KeyExchange_Request__Sequence__init(seal_msgs__srv__KeyExchange_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__srv__KeyExchange_Request * data = NULL;

  if (size) {
    data = (seal_msgs__srv__KeyExchange_Request *)allocator.zero_allocate(size, sizeof(seal_msgs__srv__KeyExchange_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = seal_msgs__srv__KeyExchange_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        seal_msgs__srv__KeyExchange_Request__fini(&data[i - 1]);
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
seal_msgs__srv__KeyExchange_Request__Sequence__fini(seal_msgs__srv__KeyExchange_Request__Sequence * array)
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
      seal_msgs__srv__KeyExchange_Request__fini(&array->data[i]);
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

seal_msgs__srv__KeyExchange_Request__Sequence *
seal_msgs__srv__KeyExchange_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__srv__KeyExchange_Request__Sequence * array = (seal_msgs__srv__KeyExchange_Request__Sequence *)allocator.allocate(sizeof(seal_msgs__srv__KeyExchange_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = seal_msgs__srv__KeyExchange_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
seal_msgs__srv__KeyExchange_Request__Sequence__destroy(seal_msgs__srv__KeyExchange_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    seal_msgs__srv__KeyExchange_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
seal_msgs__srv__KeyExchange_Request__Sequence__are_equal(const seal_msgs__srv__KeyExchange_Request__Sequence * lhs, const seal_msgs__srv__KeyExchange_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!seal_msgs__srv__KeyExchange_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
seal_msgs__srv__KeyExchange_Request__Sequence__copy(
  const seal_msgs__srv__KeyExchange_Request__Sequence * input,
  seal_msgs__srv__KeyExchange_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(seal_msgs__srv__KeyExchange_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    seal_msgs__srv__KeyExchange_Request * data =
      (seal_msgs__srv__KeyExchange_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!seal_msgs__srv__KeyExchange_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          seal_msgs__srv__KeyExchange_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!seal_msgs__srv__KeyExchange_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
seal_msgs__srv__KeyExchange_Response__init(seal_msgs__srv__KeyExchange_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    seal_msgs__srv__KeyExchange_Response__fini(msg);
    return false;
  }
  return true;
}

void
seal_msgs__srv__KeyExchange_Response__fini(seal_msgs__srv__KeyExchange_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
seal_msgs__srv__KeyExchange_Response__are_equal(const seal_msgs__srv__KeyExchange_Response * lhs, const seal_msgs__srv__KeyExchange_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
seal_msgs__srv__KeyExchange_Response__copy(
  const seal_msgs__srv__KeyExchange_Response * input,
  seal_msgs__srv__KeyExchange_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

seal_msgs__srv__KeyExchange_Response *
seal_msgs__srv__KeyExchange_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__srv__KeyExchange_Response * msg = (seal_msgs__srv__KeyExchange_Response *)allocator.allocate(sizeof(seal_msgs__srv__KeyExchange_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(seal_msgs__srv__KeyExchange_Response));
  bool success = seal_msgs__srv__KeyExchange_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
seal_msgs__srv__KeyExchange_Response__destroy(seal_msgs__srv__KeyExchange_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    seal_msgs__srv__KeyExchange_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
seal_msgs__srv__KeyExchange_Response__Sequence__init(seal_msgs__srv__KeyExchange_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__srv__KeyExchange_Response * data = NULL;

  if (size) {
    data = (seal_msgs__srv__KeyExchange_Response *)allocator.zero_allocate(size, sizeof(seal_msgs__srv__KeyExchange_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = seal_msgs__srv__KeyExchange_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        seal_msgs__srv__KeyExchange_Response__fini(&data[i - 1]);
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
seal_msgs__srv__KeyExchange_Response__Sequence__fini(seal_msgs__srv__KeyExchange_Response__Sequence * array)
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
      seal_msgs__srv__KeyExchange_Response__fini(&array->data[i]);
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

seal_msgs__srv__KeyExchange_Response__Sequence *
seal_msgs__srv__KeyExchange_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  seal_msgs__srv__KeyExchange_Response__Sequence * array = (seal_msgs__srv__KeyExchange_Response__Sequence *)allocator.allocate(sizeof(seal_msgs__srv__KeyExchange_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = seal_msgs__srv__KeyExchange_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
seal_msgs__srv__KeyExchange_Response__Sequence__destroy(seal_msgs__srv__KeyExchange_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    seal_msgs__srv__KeyExchange_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
seal_msgs__srv__KeyExchange_Response__Sequence__are_equal(const seal_msgs__srv__KeyExchange_Response__Sequence * lhs, const seal_msgs__srv__KeyExchange_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!seal_msgs__srv__KeyExchange_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
seal_msgs__srv__KeyExchange_Response__Sequence__copy(
  const seal_msgs__srv__KeyExchange_Response__Sequence * input,
  seal_msgs__srv__KeyExchange_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(seal_msgs__srv__KeyExchange_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    seal_msgs__srv__KeyExchange_Response * data =
      (seal_msgs__srv__KeyExchange_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!seal_msgs__srv__KeyExchange_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          seal_msgs__srv__KeyExchange_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!seal_msgs__srv__KeyExchange_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
