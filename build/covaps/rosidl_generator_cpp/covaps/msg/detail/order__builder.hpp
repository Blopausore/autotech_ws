// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from covaps:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef COVAPS__MSG__DETAIL__ORDER__BUILDER_HPP_
#define COVAPS__MSG__DETAIL__ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "covaps/msg/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace covaps
{

namespace msg
{

namespace builder
{

class Init_Order_type
{
public:
  explicit Init_Order_type(::covaps::msg::Order & msg)
  : msg_(msg)
  {}
  ::covaps::msg::Order type(::covaps::msg::Order::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::covaps::msg::Order msg_;
};

class Init_Order_val
{
public:
  Init_Order_val()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Order_type val(::covaps::msg::Order::_val_type arg)
  {
    msg_.val = std::move(arg);
    return Init_Order_type(msg_);
  }

private:
  ::covaps::msg::Order msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::covaps::msg::Order>()
{
  return covaps::msg::builder::Init_Order_val();
}

}  // namespace covaps

#endif  // COVAPS__MSG__DETAIL__ORDER__BUILDER_HPP_
