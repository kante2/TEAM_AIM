// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scene_srv:srv/SceneSignal.idl
// generated code does not contain a copyright notice

#ifndef SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__STRUCT_HPP_
#define SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__scene_srv__srv__SceneSignal_Request __attribute__((deprecated))
#else
# define DEPRECATED__scene_srv__srv__SceneSignal_Request __declspec(deprecated)
#endif

namespace scene_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SceneSignal_Request_
{
  using Type = SceneSignal_Request_<ContainerAllocator>;

  explicit SceneSignal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->signal = 0ll;
      this->state = 0ll;
    }
  }

  explicit SceneSignal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->signal = 0ll;
      this->state = 0ll;
    }
  }

  // field types and members
  using _signal_type =
    int64_t;
  _signal_type signal;
  using _state_type =
    int64_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__signal(
    const int64_t & _arg)
  {
    this->signal = _arg;
    return *this;
  }
  Type & set__state(
    const int64_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scene_srv::srv::SceneSignal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const scene_srv::srv::SceneSignal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scene_srv::srv::SceneSignal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scene_srv::srv::SceneSignal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scene_srv::srv::SceneSignal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scene_srv::srv::SceneSignal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scene_srv::srv::SceneSignal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scene_srv::srv::SceneSignal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scene_srv::srv::SceneSignal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scene_srv::srv::SceneSignal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scene_srv__srv__SceneSignal_Request
    std::shared_ptr<scene_srv::srv::SceneSignal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scene_srv__srv__SceneSignal_Request
    std::shared_ptr<scene_srv::srv::SceneSignal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SceneSignal_Request_ & other) const
  {
    if (this->signal != other.signal) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SceneSignal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SceneSignal_Request_

// alias to use template instance with default allocator
using SceneSignal_Request =
  scene_srv::srv::SceneSignal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace scene_srv


#ifndef _WIN32
# define DEPRECATED__scene_srv__srv__SceneSignal_Response __attribute__((deprecated))
#else
# define DEPRECATED__scene_srv__srv__SceneSignal_Response __declspec(deprecated)
#endif

namespace scene_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SceneSignal_Response_
{
  using Type = SceneSignal_Response_<ContainerAllocator>;

  explicit SceneSignal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SceneSignal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scene_srv::srv::SceneSignal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const scene_srv::srv::SceneSignal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scene_srv::srv::SceneSignal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scene_srv::srv::SceneSignal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scene_srv::srv::SceneSignal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scene_srv::srv::SceneSignal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scene_srv::srv::SceneSignal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scene_srv::srv::SceneSignal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scene_srv::srv::SceneSignal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scene_srv::srv::SceneSignal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scene_srv__srv__SceneSignal_Response
    std::shared_ptr<scene_srv::srv::SceneSignal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scene_srv__srv__SceneSignal_Response
    std::shared_ptr<scene_srv::srv::SceneSignal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SceneSignal_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SceneSignal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SceneSignal_Response_

// alias to use template instance with default allocator
using SceneSignal_Response =
  scene_srv::srv::SceneSignal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace scene_srv

namespace scene_srv
{

namespace srv
{

struct SceneSignal
{
  using Request = scene_srv::srv::SceneSignal_Request;
  using Response = scene_srv::srv::SceneSignal_Response;
};

}  // namespace srv

}  // namespace scene_srv

#endif  // SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__STRUCT_HPP_
