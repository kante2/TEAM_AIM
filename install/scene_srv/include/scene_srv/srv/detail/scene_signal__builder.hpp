// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scene_srv:srv/SceneSignal.idl
// generated code does not contain a copyright notice

#ifndef SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__BUILDER_HPP_
#define SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__BUILDER_HPP_

#include "scene_srv/srv/detail/scene_signal__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace scene_srv
{

namespace srv
{

namespace builder
{

class Init_SceneSignal_Request_state
{
public:
  explicit Init_SceneSignal_Request_state(::scene_srv::srv::SceneSignal_Request & msg)
  : msg_(msg)
  {}
  ::scene_srv::srv::SceneSignal_Request state(::scene_srv::srv::SceneSignal_Request::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scene_srv::srv::SceneSignal_Request msg_;
};

class Init_SceneSignal_Request_signal
{
public:
  Init_SceneSignal_Request_signal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SceneSignal_Request_state signal(::scene_srv::srv::SceneSignal_Request::_signal_type arg)
  {
    msg_.signal = std::move(arg);
    return Init_SceneSignal_Request_state(msg_);
  }

private:
  ::scene_srv::srv::SceneSignal_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::scene_srv::srv::SceneSignal_Request>()
{
  return scene_srv::srv::builder::Init_SceneSignal_Request_signal();
}

}  // namespace scene_srv


namespace scene_srv
{

namespace srv
{

namespace builder
{

class Init_SceneSignal_Response_message
{
public:
  explicit Init_SceneSignal_Response_message(::scene_srv::srv::SceneSignal_Response & msg)
  : msg_(msg)
  {}
  ::scene_srv::srv::SceneSignal_Response message(::scene_srv::srv::SceneSignal_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scene_srv::srv::SceneSignal_Response msg_;
};

class Init_SceneSignal_Response_success
{
public:
  Init_SceneSignal_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SceneSignal_Response_message success(::scene_srv::srv::SceneSignal_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SceneSignal_Response_message(msg_);
  }

private:
  ::scene_srv::srv::SceneSignal_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::scene_srv::srv::SceneSignal_Response>()
{
  return scene_srv::srv::builder::Init_SceneSignal_Response_success();
}

}  // namespace scene_srv

#endif  // SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__BUILDER_HPP_
