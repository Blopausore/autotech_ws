// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from covaps:msg/Order.idl
// generated code does not contain a copyright notice
#include "covaps/msg/detail/order__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `type`
#include "rosidl_runtime_c/string_functions.h"

bool
covaps__msg__Order__init(covaps__msg__Order * msg)
{
  if (!msg) {
    return false;
  }
  // val
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    covaps__msg__Order__fini(msg);
    return false;
  }
  return true;
}

void
covaps__msg__Order__fini(covaps__msg__Order * msg)
{
  if (!msg) {
    return;
  }
  // val
  // type
  rosidl_runtime_c__String__fini(&msg->type);
}

bool
covaps__msg__Order__are_equal(const covaps__msg__Order * lhs, const covaps__msg__Order * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // val
  if (lhs->val != rhs->val) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  return true;
}

bool
covaps__msg__Order__copy(
  const covaps__msg__Order * input,
  covaps__msg__Order * output)
{
  if (!input || !output) {
    return false;
  }
  // val
  output->val = input->val;
  // type
  if (!rosidl_runtime_c__String__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  return true;
}

covaps__msg__Order *
covaps__msg__Order__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  covaps__msg__Order * msg = (covaps__msg__Order *)allocator.allocate(sizeof(covaps__msg__Order), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(covaps__msg__Order));
  bool success = covaps__msg__Order__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
covaps__msg__Order__destroy(covaps__msg__Order * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    covaps__msg__Order__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
covaps__msg__Order__Sequence__init(covaps__msg__Order__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  covaps__msg__Order * data = NULL;

  if (size) {
    data = (covaps__msg__Order *)allocator.zero_allocate(size, sizeof(covaps__msg__Order), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = covaps__msg__Order__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        covaps__msg__Order__fini(&data[i - 1]);
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
covaps__msg__Order__Sequence__fini(covaps__msg__Order__Sequence * array)
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
      covaps__msg__Order__fini(&array->data[i]);
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

covaps__msg__Order__Sequence *
covaps__msg__Order__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  covaps__msg__Order__Sequence * array = (covaps__msg__Order__Sequence *)allocator.allocate(sizeof(covaps__msg__Order__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = covaps__msg__Order__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
covaps__msg__Order__Sequence__destroy(covaps__msg__Order__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    covaps__msg__Order__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
covaps__msg__Order__Sequence__are_equal(const covaps__msg__Order__Sequence * lhs, const covaps__msg__Order__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!covaps__msg__Order__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
covaps__msg__Order__Sequence__copy(
  const covaps__msg__Order__Sequence * input,
  covaps__msg__Order__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(covaps__msg__Order);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    covaps__msg__Order * data =
      (covaps__msg__Order *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!covaps__msg__Order__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          covaps__msg__Order__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!covaps__msg__Order__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
