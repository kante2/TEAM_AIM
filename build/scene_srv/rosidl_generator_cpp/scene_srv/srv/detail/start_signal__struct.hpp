// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scene_srv:srv/StartSignal.idl
// generated code does not contain a copyright notice

#ifndef SCENE_SRV__SRV__DETAIL__START_SIGNAL__STRUCT_HPP_
#define SCENE_SRV__SRV__DETAIL__START_SIGNAL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__scene_srv__srv__StartSignal_Request __attribute__((deprecated))
#else
# define DEPRECATED__scene_srv__srv__StartSignal_Request __declspec(deprecated)
#endif

namespace scene_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartSignal_Request_
{
  using Type = StartSignal_Request_<ContainerAllocator>;

  explicit StartSignal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->new_event = 0ll;
      this->scene_number = 0ll;
    }
  }

  explicit StartSignal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->new_event = 0ll;
      this->scene_number = 0ll;
    }
  }

  // field types and members
  using _new_event_type =
    int64_t;
  _new_event_type new_event;
  using _scene_number_type =
    int64_t;
  _scene_number_type scene_number;

  // setters for named parameter idiom
  Type & set__new_event(
    const int64_t & _arg)
  {
    this->new_event = _arg;
    return *this;
  }
  Type & set__scene_number(
    const int64_t & _arg)
  {
    this->scene_number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scene_srv::srv::StartSignal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const scene_srv::srv::StartSignal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scene_srv::srv::StartSignal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scene_srv::srv::StartSignal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scene_srv::srv::StartSignal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scene_srv::srv::StartSignal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scene_srv::srv::StartSignal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scene_srv::srv::StartSignal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scene_srv::srv::StartSignal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scene_srv::srv::StartSignal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scene_srv__srv__StartSignal_Request
    std::shared_ptr<scene_srv::srv::StartSignal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scene_srv__srv__StartSignal_Request
    std::shared_ptr<scene_srv::srv::StartSignal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartSignal_Request_ & other) const
  {
    if (this->new_event != other.new_event) {
      return false;
    }
    if (this->scene_number != other.scene_number) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartSignal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartSignal_Request_

// alias to use template instance with default allocator
using StartSignal_Request =
  scene_srv::srv::StartSignal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace scene_srv


#ifndef _WIN32
# define DEPRECATED__scene_srv__srv__StartSignal_Response __attribute__((deprecated))
#else
# define DEPRECATED__scene_srv__srv__StartSignal_Response __declspec(deprecated)
#endif

namespace scene_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartSignal_Response_
{
  using Type = StartSignal_Response_<ContainerAllocator>;

  explicit StartSignal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit StartSignal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scene_srv::srv::StartSignal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const scene_srv::srv::StartSignal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scene_srv::srv::StartSignal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scene_srv::srv::StartSignal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scene_srv::srv::StartSignal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scene_srv::srv::StartSignal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scene_srv::srv::StartSignal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scene_srv::srv::StartSignal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scene_srv::srv::StartSignal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scene_srv::srv::StartSignal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scene_srv__srv__StartSignal_Response
    std::shared_ptr<scene_srv::srv::StartSignal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scene_srv__srv__StartSignal_Response
    std::shared_ptr<scene_srv::srv::StartSignal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartSignal_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartSignal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartSignal_Response_

// alias to use template instance with default allocator
using StartSignal_Response =
  scene_srv::srv::StartSignal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace scene_srv

namespace scene_srv
{

namespace srv
{

struct StartSignal
{
  using Request = scene_srv::srv::StartSignal_Request;
  using Response = scene_srv::srv::StartSignal_Response;
};

}  // namespace srv

}  // namespace scene_srv

#endif  // SCENE_SRV__SRV__DETAIL__START_SIGNAL__STRUCT_HPP_
