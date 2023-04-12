// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from covaps:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef COVAPS__MSG__DETAIL__ORDER__STRUCT_HPP_
#define COVAPS__MSG__DETAIL__ORDER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__covaps__msg__Order __attribute__((deprecated))
#else
# define DEPRECATED__covaps__msg__Order __declspec(deprecated)
#endif

namespace covaps
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Order_
{
  using Type = Order_<ContainerAllocator>;

  explicit Order_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->val = 0ll;
      this->type = "";
    }
  }

  explicit Order_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->val = 0ll;
      this->type = "";
    }
  }

  // field types and members
  using _val_type =
    int64_t;
  _val_type val;
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _type_type type;

  // setters for named parameter idiom
  Type & set__val(
    const int64_t & _arg)
  {
    this->val = _arg;
    return *this;
  }
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    covaps::msg::Order_<ContainerAllocator> *;
  using ConstRawPtr =
    const covaps::msg::Order_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<covaps::msg::Order_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<covaps::msg::Order_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      covaps::msg::Order_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<covaps::msg::Order_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      covaps::msg::Order_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<covaps::msg::Order_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<covaps::msg::Order_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<covaps::msg::Order_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__covaps__msg__Order
    std::shared_ptr<covaps::msg::Order_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__covaps__msg__Order
    std::shared_ptr<covaps::msg::Order_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Order_ & other) const
  {
    if (this->val != other.val) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const Order_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Order_

// alias to use template instance with default allocator
using Order =
  covaps::msg::Order_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace covaps

#endif  // COVAPS__MSG__DETAIL__ORDER__STRUCT_HPP_
