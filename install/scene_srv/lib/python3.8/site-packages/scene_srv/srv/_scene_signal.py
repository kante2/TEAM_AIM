# generated from rosidl_generator_py/resource/_idl.py.em
# with input from scene_srv:srv/SceneSignal.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SceneSignal_Request(type):
    """Metaclass of message 'SceneSignal_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('scene_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'scene_srv.srv.SceneSignal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__scene_signal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__scene_signal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__scene_signal__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__scene_signal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__scene_signal__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SceneSignal_Request(metaclass=Metaclass_SceneSignal_Request):
    """Message class 'SceneSignal_Request'."""

    __slots__ = [
        '_signal',
        '_state',
    ]

    _fields_and_field_types = {
        'signal': 'int64',
        'state': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.signal = kwargs.get('signal', int())
        self.state = kwargs.get('state', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.signal != other.signal:
            return False
        if self.state != other.state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def signal(self):
        """Message field 'signal'."""
        return self._signal

    @signal.setter
    def signal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'signal' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'signal' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._signal = value

    @property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'state' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._state = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_SceneSignal_Response(type):
    """Metaclass of message 'SceneSignal_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('scene_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'scene_srv.srv.SceneSignal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__scene_signal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__scene_signal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__scene_signal__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__scene_signal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__scene_signal__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SceneSignal_Response(metaclass=Metaclass_SceneSignal_Response):
    """Message class 'SceneSignal_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_SceneSignal(type):
    """Metaclass of service 'SceneSignal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('scene_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'scene_srv.srv.SceneSignal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__scene_signal

            from scene_srv.srv import _scene_signal
            if _scene_signal.Metaclass_SceneSignal_Request._TYPE_SUPPORT is None:
                _scene_signal.Metaclass_SceneSignal_Request.__import_type_support__()
            if _scene_signal.Metaclass_SceneSignal_Response._TYPE_SUPPORT is None:
                _scene_signal.Metaclass_SceneSignal_Response.__import_type_support__()


class SceneSignal(metaclass=Metaclass_SceneSignal):
    from scene_srv.srv._scene_signal import SceneSignal_Request as Request
    from scene_srv.srv._scene_signal import SceneSignal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
