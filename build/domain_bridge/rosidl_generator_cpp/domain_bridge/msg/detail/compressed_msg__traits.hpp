// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from domain_bridge:msg/CompressedMsg.idl
// generated code does not contain a copyright notice

#ifndef DOMAIN_BRIDGE__MSG__DETAIL__COMPRESSED_MSG__TRAITS_HPP_
#define DOMAIN_BRIDGE__MSG__DETAIL__COMPRESSED_MSG__TRAITS_HPP_

#include "domain_bridge/msg/detail/compressed_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<domain_bridge::msg::CompressedMsg>()
{
  return "domain_bridge::msg::CompressedMsg";
}

template<>
inline const char * name<domain_bridge::msg::CompressedMsg>()
{
  return "domain_bridge/msg/CompressedMsg";
}

template<>
struct has_fixed_size<domain_bridge::msg::CompressedMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<domain_bridge::msg::CompressedMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<domain_bridge::msg::CompressedMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DOMAIN_BRIDGE__MSG__DETAIL__COMPRESSED_MSG__TRAITS_HPP_
