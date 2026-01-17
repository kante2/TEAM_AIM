// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from scene_srv:srv/SceneSignal.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "scene_srv/srv/detail/scene_signal__rosidl_typesupport_introspection_c.h"
#include "scene_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "scene_srv/srv/detail/scene_signal__functions.h"
#include "scene_srv/srv/detail/scene_signal__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scene_srv__srv__SceneSignal_Request__init(message_memory);
}

void SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_fini_function(void * message_memory)
{
  scene_srv__srv__SceneSignal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_message_member_array[2] = {
  {
    "signal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_srv__srv__SceneSignal_Request, signal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_srv__srv__SceneSignal_Request, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_message_members = {
  "scene_srv__srv",  // message namespace
  "SceneSignal_Request",  // message name
  2,  // number of fields
  sizeof(scene_srv__srv__SceneSignal_Request),
  SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_message_member_array,  // message members
  SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_message_type_support_handle = {
  0,
  &SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scene_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, SceneSignal_Request)() {
  if (!SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_message_type_support_handle.typesupport_identifier) {
    SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SceneSignal_Request__rosidl_typesupport_introspection_c__SceneSignal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "scene_srv/srv/detail/scene_signal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "scene_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "scene_srv/srv/detail/scene_signal__functions.h"
// already included above
// #include "scene_srv/srv/detail/scene_signal__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scene_srv__srv__SceneSignal_Response__init(message_memory);
}

void SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_fini_function(void * message_memory)
{
  scene_srv__srv__SceneSignal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_srv__srv__SceneSignal_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_srv__srv__SceneSignal_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_message_members = {
  "scene_srv__srv",  // message namespace
  "SceneSignal_Response",  // message name
  2,  // number of fields
  sizeof(scene_srv__srv__SceneSignal_Response),
  SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_message_member_array,  // message members
  SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_message_type_support_handle = {
  0,
  &SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scene_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, SceneSignal_Response)() {
  if (!SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_message_type_support_handle.typesupport_identifier) {
    SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SceneSignal_Response__rosidl_typesupport_introspection_c__SceneSignal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "scene_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "scene_srv/srv/detail/scene_signal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_service_members = {
  "scene_srv__srv",  // service namespace
  "SceneSignal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_Request_message_type_support_handle,
  NULL  // response message
  // scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_Response_message_type_support_handle
};

static rosidl_service_type_support_t scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_service_type_support_handle = {
  0,
  &scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, SceneSignal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, SceneSignal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scene_srv
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, SceneSignal)() {
  if (!scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_service_type_support_handle.typesupport_identifier) {
    scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, SceneSignal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, SceneSignal_Response)()->data;
  }

  return &scene_srv__srv__detail__scene_signal__rosidl_typesupport_introspection_c__SceneSignal_service_type_support_handle;
}
