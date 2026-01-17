// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scene_srv:srv/SceneSignal.idl
// generated code does not contain a copyright notice
#include "scene_srv/srv/detail/scene_signal__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
scene_srv__srv__SceneSignal_Request__init(scene_srv__srv__SceneSignal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // signal
  // state
  return true;
}

void
scene_srv__srv__SceneSignal_Request__fini(scene_srv__srv__SceneSignal_Request * msg)
{
  if (!msg) {
    return;
  }
  // signal
  // state
}

bool
scene_srv__srv__SceneSignal_Request__are_equal(const scene_srv__srv__SceneSignal_Request * lhs, const scene_srv__srv__SceneSignal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // signal
  if (lhs->signal != rhs->signal) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
scene_srv__srv__SceneSignal_Request__copy(
  const scene_srv__srv__SceneSignal_Request * input,
  scene_srv__srv__SceneSignal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // signal
  output->signal = input->signal;
  // state
  output->state = input->state;
  return true;
}

scene_srv__srv__SceneSignal_Request *
scene_srv__srv__SceneSignal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_srv__srv__SceneSignal_Request * msg = (scene_srv__srv__SceneSignal_Request *)allocator.allocate(sizeof(scene_srv__srv__SceneSignal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scene_srv__srv__SceneSignal_Request));
  bool success = scene_srv__srv__SceneSignal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scene_srv__srv__SceneSignal_Request__destroy(scene_srv__srv__SceneSignal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scene_srv__srv__SceneSignal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scene_srv__srv__SceneSignal_Request__Sequence__init(scene_srv__srv__SceneSignal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_srv__srv__SceneSignal_Request * data = NULL;

  if (size) {
    data = (scene_srv__srv__SceneSignal_Request *)allocator.zero_allocate(size, sizeof(scene_srv__srv__SceneSignal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scene_srv__srv__SceneSignal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scene_srv__srv__SceneSignal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
scene_srv__srv__SceneSignal_Request__Sequence__fini(scene_srv__srv__SceneSignal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      scene_srv__srv__SceneSignal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

scene_srv__srv__SceneSignal_Request__Sequence *
scene_srv__srv__SceneSignal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_srv__srv__SceneSignal_Request__Sequence * array = (scene_srv__srv__SceneSignal_Request__Sequence *)allocator.allocate(sizeof(scene_srv__srv__SceneSignal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scene_srv__srv__SceneSignal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scene_srv__srv__SceneSignal_Request__Sequence__destroy(scene_srv__srv__SceneSignal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scene_srv__srv__SceneSignal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scene_srv__srv__SceneSignal_Request__Sequence__are_equal(const scene_srv__srv__SceneSignal_Request__Sequence * lhs, const scene_srv__srv__SceneSignal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scene_srv__srv__SceneSignal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scene_srv__srv__SceneSignal_Request__Sequence__copy(
  const scene_srv__srv__SceneSignal_Request__Sequence * input,
  scene_srv__srv__SceneSignal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scene_srv__srv__SceneSignal_Request);
    scene_srv__srv__SceneSignal_Request * data =
      (scene_srv__srv__SceneSignal_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scene_srv__srv__SceneSignal_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          scene_srv__srv__SceneSignal_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scene_srv__srv__SceneSignal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
scene_srv__srv__SceneSignal_Response__init(scene_srv__srv__SceneSignal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    scene_srv__srv__SceneSignal_Response__fini(msg);
    return false;
  }
  return true;
}

void
scene_srv__srv__SceneSignal_Response__fini(scene_srv__srv__SceneSignal_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
scene_srv__srv__SceneSignal_Response__are_equal(const scene_srv__srv__SceneSignal_Response * lhs, const scene_srv__srv__SceneSignal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
scene_srv__srv__SceneSignal_Response__copy(
  const scene_srv__srv__SceneSignal_Response * input,
  scene_srv__srv__SceneSignal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

scene_srv__srv__SceneSignal_Response *
scene_srv__srv__SceneSignal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_srv__srv__SceneSignal_Response * msg = (scene_srv__srv__SceneSignal_Response *)allocator.allocate(sizeof(scene_srv__srv__SceneSignal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scene_srv__srv__SceneSignal_Response));
  bool success = scene_srv__srv__SceneSignal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scene_srv__srv__SceneSignal_Response__destroy(scene_srv__srv__SceneSignal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scene_srv__srv__SceneSignal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scene_srv__srv__SceneSignal_Response__Sequence__init(scene_srv__srv__SceneSignal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_srv__srv__SceneSignal_Response * data = NULL;

  if (size) {
    data = (scene_srv__srv__SceneSignal_Response *)allocator.zero_allocate(size, sizeof(scene_srv__srv__SceneSignal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scene_srv__srv__SceneSignal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scene_srv__srv__SceneSignal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
scene_srv__srv__SceneSignal_Response__Sequence__fini(scene_srv__srv__SceneSignal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      scene_srv__srv__SceneSignal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

scene_srv__srv__SceneSignal_Response__Sequence *
scene_srv__srv__SceneSignal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scene_srv__srv__SceneSignal_Response__Sequence * array = (scene_srv__srv__SceneSignal_Response__Sequence *)allocator.allocate(sizeof(scene_srv__srv__SceneSignal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scene_srv__srv__SceneSignal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scene_srv__srv__SceneSignal_Response__Sequence__destroy(scene_srv__srv__SceneSignal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scene_srv__srv__SceneSignal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scene_srv__srv__SceneSignal_Response__Sequence__are_equal(const scene_srv__srv__SceneSignal_Response__Sequence * lhs, const scene_srv__srv__SceneSignal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scene_srv__srv__SceneSignal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scene_srv__srv__SceneSignal_Response__Sequence__copy(
  const scene_srv__srv__SceneSignal_Response__Sequence * input,
  scene_srv__srv__SceneSignal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scene_srv__srv__SceneSignal_Response);
    scene_srv__srv__SceneSignal_Response * data =
      (scene_srv__srv__SceneSignal_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scene_srv__srv__SceneSignal_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          scene_srv__srv__SceneSignal_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scene_srv__srv__SceneSignal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
