// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scene_srv:srv/StartSignal.idl
// generated code does not contain a copyright notice

#ifndef SCENE_SRV__SRV__DETAIL__START_SIGNAL__TRAITS_HPP_
#define SCENE_SRV__SRV__DETAIL__START_SIGNAL__TRAITS_HPP_

#include "scene_srv/srv/detail/start_signal__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scene_srv::srv::StartSignal_Request>()
{
  return "scene_srv::srv::StartSignal_Request";
}

template<>
inline const char * name<scene_srv::srv::StartSignal_Request>()
{
  return "scene_srv/srv/StartSignal_Request";
}

template<>
struct has_fixed_size<scene_srv::srv::StartSignal_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<scene_srv::srv::StartSignal_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<scene_srv::srv::StartSignal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scene_srv::srv::StartSignal_Response>()
{
  return "scene_srv::srv::StartSignal_Response";
}

template<>
inline const char * name<scene_srv::srv::StartSignal_Response>()
{
  return "scene_srv/srv/StartSignal_Response";
}

template<>
struct has_fixed_size<scene_srv::srv::StartSignal_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<scene_srv::srv::StartSignal_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<scene_srv::srv::StartSignal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scene_srv::srv::StartSignal>()
{
  return "scene_srv::srv::StartSignal";
}

template<>
inline const char * name<scene_srv::srv::StartSignal>()
{
  return "scene_srv/srv/StartSignal";
}

template<>
struct has_fixed_size<scene_srv::srv::StartSignal>
  : std::integral_constant<
    bool,
    has_fixed_size<scene_srv::srv::StartSignal_Request>::value &&
    has_fixed_size<scene_srv::srv::StartSignal_Response>::value
  >
{
};

template<>
struct has_bounded_size<scene_srv::srv::StartSignal>
  : std::integral_constant<
    bool,
    has_bounded_size<scene_srv::srv::StartSignal_Request>::value &&
    has_bounded_size<scene_srv::srv::StartSignal_Response>::value
  >
{
};

template<>
struct is_service<scene_srv::srv::StartSignal>
  : std::true_type
{
};

template<>
struct is_service_request<scene_srv::srv::StartSignal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<scene_srv::srv::StartSignal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SCENE_SRV__SRV__DETAIL__START_SIGNAL__TRAITS_HPP_
