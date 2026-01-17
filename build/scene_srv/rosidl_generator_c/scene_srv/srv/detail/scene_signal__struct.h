// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scene_srv:srv/SceneSignal.idl
// generated code does not contain a copyright notice

#ifndef SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__STRUCT_H_
#define SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/SceneSignal in the package scene_srv.
typedef struct scene_srv__srv__SceneSignal_Request
{
  int64_t signal;
  int64_t state;
} scene_srv__srv__SceneSignal_Request;

// Struct for a sequence of scene_srv__srv__SceneSignal_Request.
typedef struct scene_srv__srv__SceneSignal_Request__Sequence
{
  scene_srv__srv__SceneSignal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scene_srv__srv__SceneSignal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SceneSignal in the package scene_srv.
typedef struct scene_srv__srv__SceneSignal_Response
{
  bool success;
  rosidl_runtime_c__String message;
} scene_srv__srv__SceneSignal_Response;

// Struct for a sequence of scene_srv__srv__SceneSignal_Response.
typedef struct scene_srv__srv__SceneSignal_Response__Sequence
{
  scene_srv__srv__SceneSignal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scene_srv__srv__SceneSignal_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCENE_SRV__SRV__DETAIL__SCENE_SIGNAL__STRUCT_H_
