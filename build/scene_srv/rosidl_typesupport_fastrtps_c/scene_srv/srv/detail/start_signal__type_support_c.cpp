// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from scene_srv:srv/StartSignal.idl
// generated code does not contain a copyright notice
#include "scene_srv/srv/detail/start_signal__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "scene_srv/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scene_srv/srv/detail/start_signal__struct.h"
#include "scene_srv/srv/detail/start_signal__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _StartSignal_Request__ros_msg_type = scene_srv__srv__StartSignal_Request;

static bool _StartSignal_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _StartSignal_Request__ros_msg_type * ros_message = static_cast<const _StartSignal_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: new_event
  {
    cdr << ros_message->new_event;
  }

  // Field name: scene_number
  {
    cdr << ros_message->scene_number;
  }

  return true;
}

static bool _StartSignal_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _StartSignal_Request__ros_msg_type * ros_message = static_cast<_StartSignal_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: new_event
  {
    cdr >> ros_message->new_event;
  }

  // Field name: scene_number
  {
    cdr >> ros_message->scene_number;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scene_srv
size_t get_serialized_size_scene_srv__srv__StartSignal_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _StartSignal_Request__ros_msg_type * ros_message = static_cast<const _StartSignal_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name new_event
  {
    size_t item_size = sizeof(ros_message->new_event);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scene_number
  {
    size_t item_size = sizeof(ros_message->scene_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _StartSignal_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scene_srv__srv__StartSignal_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scene_srv
size_t max_serialized_size_scene_srv__srv__StartSignal_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: new_event
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: scene_number
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _StartSignal_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_scene_srv__srv__StartSignal_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_StartSignal_Request = {
  "scene_srv::srv",
  "StartSignal_Request",
  _StartSignal_Request__cdr_serialize,
  _StartSignal_Request__cdr_deserialize,
  _StartSignal_Request__get_serialized_size,
  _StartSignal_Request__max_serialized_size
};

static rosidl_message_type_support_t _StartSignal_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_StartSignal_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, StartSignal_Request)() {
  return &_StartSignal_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "scene_srv/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "scene_srv/srv/detail/start_signal__struct.h"
// already included above
// #include "scene_srv/srv/detail/start_signal__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _StartSignal_Response__ros_msg_type = scene_srv__srv__StartSignal_Response;

static bool _StartSignal_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _StartSignal_Response__ros_msg_type * ros_message = static_cast<const _StartSignal_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _StartSignal_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _StartSignal_Response__ros_msg_type * ros_message = static_cast<_StartSignal_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scene_srv
size_t get_serialized_size_scene_srv__srv__StartSignal_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _StartSignal_Response__ros_msg_type * ros_message = static_cast<const _StartSignal_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _StartSignal_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scene_srv__srv__StartSignal_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scene_srv
size_t max_serialized_size_scene_srv__srv__StartSignal_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: success
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _StartSignal_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_scene_srv__srv__StartSignal_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_StartSignal_Response = {
  "scene_srv::srv",
  "StartSignal_Response",
  _StartSignal_Response__cdr_serialize,
  _StartSignal_Response__cdr_deserialize,
  _StartSignal_Response__get_serialized_size,
  _StartSignal_Response__max_serialized_size
};

static rosidl_message_type_support_t _StartSignal_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_StartSignal_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, StartSignal_Response)() {
  return &_StartSignal_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "scene_srv/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scene_srv/srv/start_signal.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t StartSignal__callbacks = {
  "scene_srv::srv",
  "StartSignal",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, StartSignal_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, StartSignal_Response)(),
};

static rosidl_service_type_support_t StartSignal__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &StartSignal__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, StartSignal)() {
  return &StartSignal__handle;
}

#if defined(__cplusplus)
}
#endif
