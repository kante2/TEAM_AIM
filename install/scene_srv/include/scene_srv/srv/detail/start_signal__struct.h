// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scene_srv:srv/StartSignal.idl
// generated code does not contain a copyright notice

#ifndef SCENE_SRV__SRV__DETAIL__START_SIGNAL__STRUCT_H_
#define SCENE_SRV__SRV__DETAIL__START_SIGNAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/StartSignal in the package scene_srv.
typedef struct scene_srv__srv__StartSignal_Request
{
  int64_t new_event;
  int64_t scene_number;
} scene_srv__srv__StartSignal_Request;

// Struct for a sequence of scene_srv__srv__StartSignal_Request.
typedef struct scene_srv__srv__StartSignal_Request__Sequence
{
  scene_srv__srv__StartSignal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scene_srv__srv__StartSignal_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/StartSignal in the package scene_srv.
typedef struct scene_srv__srv__StartSignal_Response
{
  bool success;
} scene_srv__srv__StartSignal_Response;

// Struct for a sequence of scene_srv__srv__StartSignal_Response.
typedef struct scene_srv__srv__StartSignal_Response__Sequence
{
  scene_srv__srv__StartSignal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scene_srv__srv__StartSignal_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCENE_SRV__SRV__DETAIL__START_SIGNAL__STRUCT_H_
