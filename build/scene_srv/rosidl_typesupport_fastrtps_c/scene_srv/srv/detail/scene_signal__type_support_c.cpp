// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from scene_srv:srv/SceneSignal.idl
// generated code does not contain a copyright notice
#include "scene_srv/srv/detail/scene_signal__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "scene_srv/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scene_srv/srv/detail/scene_signal__struct.h"
#include "scene_srv/srv/detail/scene_signal__functions.h"
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


using _SceneSignal_Request__ros_msg_type = scene_srv__srv__SceneSignal_Request;

static bool _SceneSignal_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SceneSignal_Request__ros_msg_type * ros_message = static_cast<const _SceneSignal_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: signal
  {
    cdr << ros_message->signal;
  }

  // Field name: state
  {
    cdr << ros_message->state;
  }

  return true;
}

static bool _SceneSignal_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SceneSignal_Request__ros_msg_type * ros_message = static_cast<_SceneSignal_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: signal
  {
    cdr >> ros_message->signal;
  }

  // Field name: state
  {
    cdr >> ros_message->state;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scene_srv
size_t get_serialized_size_scene_srv__srv__SceneSignal_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SceneSignal_Request__ros_msg_type * ros_message = static_cast<const _SceneSignal_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name signal
  {
    size_t item_size = sizeof(ros_message->signal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name state
  {
    size_t item_size = sizeof(ros_message->state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SceneSignal_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scene_srv__srv__SceneSignal_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scene_srv
size_t max_serialized_size_scene_srv__srv__SceneSignal_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: signal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _SceneSignal_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_scene_srv__srv__SceneSignal_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SceneSignal_Request = {
  "scene_srv::srv",
  "SceneSignal_Request",
  _SceneSignal_Request__cdr_serialize,
  _SceneSignal_Request__cdr_deserialize,
  _SceneSignal_Request__get_serialized_size,
  _SceneSignal_Request__max_serialized_size
};

static rosidl_message_type_support_t _SceneSignal_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SceneSignal_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, SceneSignal_Request)() {
  return &_SceneSignal_Request__type_support;
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
// #include "scene_srv/srv/detail/scene_signal__struct.h"
// already included above
// #include "scene_srv/srv/detail/scene_signal__functions.h"
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

#include "rosidl_runtime_c/string.h"  // message
#include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions


using _SceneSignal_Response__ros_msg_type = scene_srv__srv__SceneSignal_Response;

static bool _SceneSignal_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SceneSignal_Response__ros_msg_type * ros_message = static_cast<const _SceneSignal_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _SceneSignal_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SceneSignal_Response__ros_msg_type * ros_message = static_cast<_SceneSignal_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scene_srv
size_t get_serialized_size_scene_srv__srv__SceneSignal_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SceneSignal_Response__ros_msg_type * ros_message = static_cast<const _SceneSignal_Response__ros_msg_type *>(untyped_ros_message);
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
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _SceneSignal_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scene_srv__srv__SceneSignal_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scene_srv
size_t max_serialized_size_scene_srv__srv__SceneSignal_Response(
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
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _SceneSignal_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_scene_srv__srv__SceneSignal_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SceneSignal_Response = {
  "scene_srv::srv",
  "SceneSignal_Response",
  _SceneSignal_Response__cdr_serialize,
  _SceneSignal_Response__cdr_deserialize,
  _SceneSignal_Response__get_serialized_size,
  _SceneSignal_Response__max_serialized_size
};

static rosidl_message_type_support_t _SceneSignal_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SceneSignal_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, SceneSignal_Response)() {
  return &_SceneSignal_Response__type_support;
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
#include "scene_srv/srv/scene_signal.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SceneSignal__callbacks = {
  "scene_srv::srv",
  "SceneSignal",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, SceneSignal_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, SceneSignal_Response)(),
};

static rosidl_service_type_support_t SceneSignal__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SceneSignal__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scene_srv, srv, SceneSignal)() {
  return &SceneSignal__handle;
}

#if defined(__cplusplus)
}
#endif
