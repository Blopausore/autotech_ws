// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from covaps:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef COVAPS__MSG__DETAIL__ORDER__STRUCT_H_
#define COVAPS__MSG__DETAIL__ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Order in the package covaps.
typedef struct covaps__msg__Order
{
  int64_t val;
  rosidl_runtime_c__String type;
} covaps__msg__Order;

// Struct for a sequence of covaps__msg__Order.
typedef struct covaps__msg__Order__Sequence
{
  covaps__msg__Order * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} covaps__msg__Order__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COVAPS__MSG__DETAIL__ORDER__STRUCT_H_
