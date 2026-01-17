// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from scene_srv:srv/StartSignal.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "scene_srv/srv/detail/start_signal__rosidl_typesupport_introspection_c.h"
#include "scene_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "scene_srv/srv/detail/start_signal__functions.h"
#include "scene_srv/srv/detail/start_signal__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scene_srv__srv__StartSignal_Request__init(message_memory);
}

void StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_fini_function(void * message_memory)
{
  scene_srv__srv__StartSignal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_message_member_array[2] = {
  {
    "new_event",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_srv__srv__StartSignal_Request, new_event),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scene_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_srv__srv__StartSignal_Request, scene_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_message_members = {
  "scene_srv__srv",  // message namespace
  "StartSignal_Request",  // message name
  2,  // number of fields
  sizeof(scene_srv__srv__StartSignal_Request),
  StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_message_member_array,  // message members
  StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_message_type_support_handle = {
  0,
  &StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scene_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, StartSignal_Request)() {
  if (!StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_message_type_support_handle.typesupport_identifier) {
    StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &StartSignal_Request__rosidl_typesupport_introspection_c__StartSignal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "scene_srv/srv/detail/start_signal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "scene_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "scene_srv/srv/detail/start_signal__functions.h"
// already included above
// #include "scene_srv/srv/detail/start_signal__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scene_srv__srv__StartSignal_Response__init(message_memory);
}

void StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_fini_function(void * message_memory)
{
  scene_srv__srv__StartSignal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scene_srv__srv__StartSignal_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_message_members = {
  "scene_srv__srv",  // message namespace
  "StartSignal_Response",  // message name
  1,  // number of fields
  sizeof(scene_srv__srv__StartSignal_Response),
  StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_message_member_array,  // message members
  StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_message_type_support_handle = {
  0,
  &StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scene_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, StartSignal_Response)() {
  if (!StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_message_type_support_handle.typesupport_identifier) {
    StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &StartSignal_Response__rosidl_typesupport_introspection_c__StartSignal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "scene_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "scene_srv/srv/detail/start_signal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_service_members = {
  "scene_srv__srv",  // service namespace
  "StartSignal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_Request_message_type_support_handle,
  NULL  // response message
  // scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_Response_message_type_support_handle
};

static rosidl_service_type_support_t scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_service_type_support_handle = {
  0,
  &scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, StartSignal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, StartSignal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scene_srv
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, StartSignal)() {
  if (!scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_service_type_support_handle.typesupport_identifier) {
    scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, StartSignal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scene_srv, srv, StartSignal_Response)()->data;
  }

  return &scene_srv__srv__detail__start_signal__rosidl_typesupport_introspection_c__StartSignal_service_type_support_handle;
}
