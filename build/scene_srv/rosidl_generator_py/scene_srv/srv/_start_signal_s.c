// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from scene_srv:srv/StartSignal.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "scene_srv/srv/detail/start_signal__struct.h"
#include "scene_srv/srv/detail/start_signal__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool scene_srv__srv__start_signal__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("scene_srv.srv._start_signal.StartSignal_Request", full_classname_dest, 47) == 0);
  }
  scene_srv__srv__StartSignal_Request * ros_message = _ros_message;
  {  // new_event
    PyObject * field = PyObject_GetAttrString(_pymsg, "new_event");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->new_event = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // scene_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "scene_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->scene_number = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * scene_srv__srv__start_signal__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartSignal_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("scene_srv.srv._start_signal");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartSignal_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  scene_srv__srv__StartSignal_Request * ros_message = (scene_srv__srv__StartSignal_Request *)raw_ros_message;
  {  // new_event
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->new_event);
    {
      int rc = PyObject_SetAttrString(_pymessage, "new_event", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // scene_number
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->scene_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "scene_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "scene_srv/srv/detail/start_signal__struct.h"
// already included above
// #include "scene_srv/srv/detail/start_signal__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool scene_srv__srv__start_signal__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("scene_srv.srv._start_signal.StartSignal_Response", full_classname_dest, 48) == 0);
  }
  scene_srv__srv__StartSignal_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * scene_srv__srv__start_signal__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartSignal_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("scene_srv.srv._start_signal");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartSignal_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  scene_srv__srv__StartSignal_Response * ros_message = (scene_srv__srv__StartSignal_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
