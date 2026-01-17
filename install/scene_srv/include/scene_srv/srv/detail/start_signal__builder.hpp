// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scene_srv:srv/StartSignal.idl
// generated code does not contain a copyright notice

#ifndef SCENE_SRV__SRV__DETAIL__START_SIGNAL__BUILDER_HPP_
#define SCENE_SRV__SRV__DETAIL__START_SIGNAL__BUILDER_HPP_

#include "scene_srv/srv/detail/start_signal__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace scene_srv
{

namespace srv
{

namespace builder
{

class Init_StartSignal_Request_scene_number
{
public:
  explicit Init_StartSignal_Request_scene_number(::scene_srv::srv::StartSignal_Request & msg)
  : msg_(msg)
  {}
  ::scene_srv::srv::StartSignal_Request scene_number(::scene_srv::srv::StartSignal_Request::_scene_number_type arg)
  {
    msg_.scene_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scene_srv::srv::StartSignal_Request msg_;
};

class Init_StartSignal_Request_new_event
{
public:
  Init_StartSignal_Request_new_event()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartSignal_Request_scene_number new_event(::scene_srv::srv::StartSignal_Request::_new_event_type arg)
  {
    msg_.new_event = std::move(arg);
    return Init_StartSignal_Request_scene_number(msg_);
  }

private:
  ::scene_srv::srv::StartSignal_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::scene_srv::srv::StartSignal_Request>()
{
  return scene_srv::srv::builder::Init_StartSignal_Request_new_event();
}

}  // namespace scene_srv


namespace scene_srv
{

namespace srv
{

namespace builder
{

class Init_StartSignal_Response_success
{
public:
  Init_StartSignal_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::scene_srv::srv::StartSignal_Response success(::scene_srv::srv::StartSignal_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scene_srv::srv::StartSignal_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::scene_srv::srv::StartSignal_Response>()
{
  return scene_srv::srv::builder::Init_StartSignal_Response_success();
}

}  // namespace scene_srv

#endif  // SCENE_SRV__SRV__DETAIL__START_SIGNAL__BUILDER_HPP_
