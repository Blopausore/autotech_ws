// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from covaps:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef COVAPS__MSG__DETAIL__ORDER__TRAITS_HPP_
#define COVAPS__MSG__DETAIL__ORDER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "covaps/msg/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace covaps
{

namespace msg
{

inline void to_flow_style_yaml(
  const Order & msg,
  std::ostream & out)
{
  out << "{";
  // member: val
  {
    out << "val: ";
    rosidl_generator_traits::value_to_yaml(msg.val, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Order & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: val
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "val: ";
    rosidl_generator_traits::value_to_yaml(msg.val, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Order & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace covaps

namespace rosidl_generator_traits
{

[[deprecated("use covaps::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const covaps::msg::Order & msg,
  std::ostream & out, size_t indentation = 0)
{
  covaps::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use covaps::msg::to_yaml() instead")]]
inline std::string to_yaml(const covaps::msg::Order & msg)
{
  return covaps::msg::to_yaml(msg);
}

template<>
inline const char * data_type<covaps::msg::Order>()
{
  return "covaps::msg::Order";
}

template<>
inline const char * name<covaps::msg::Order>()
{
  return "covaps/msg/Order";
}

template<>
struct has_fixed_size<covaps::msg::Order>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<covaps::msg::Order>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<covaps::msg::Order>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COVAPS__MSG__DETAIL__ORDER__TRAITS_HPP_
