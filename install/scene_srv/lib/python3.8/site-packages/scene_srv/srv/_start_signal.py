# generated from rosidl_generator_py/resource/_idl.py.em
# with input from scene_srv:srv/StartSignal.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StartSignal_Request(type):
    """Metaclass of message 'StartSignal_Request'."""

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
                'scene_srv.srv.StartSignal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_signal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_signal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_signal__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_signal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_signal__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StartSignal_Request(metaclass=Metaclass_StartSignal_Request):
    """Message class 'StartSignal_Request'."""

    __slots__ = [
        '_new_event',
        '_scene_number',
    ]

    _fields_and_field_types = {
        'new_event': 'int64',
        'scene_number': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.new_event = kwargs.get('new_event', int())
        self.scene_number = kwargs.get('scene_number', int())

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
        if self.new_event != other.new_event:
            return False
        if self.scene_number != other.scene_number:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def new_event(self):
        """Message field 'new_event'."""
        return self._new_event

    @new_event.setter
    def new_event(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'new_event' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'new_event' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._new_event = value

    @property
    def scene_number(self):
        """Message field 'scene_number'."""
        return self._scene_number

    @scene_number.setter
    def scene_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'scene_number' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'scene_number' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._scene_number = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_StartSignal_Response(type):
    """Metaclass of message 'StartSignal_Response'."""

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
                'scene_srv.srv.StartSignal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_signal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_signal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_signal__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_signal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_signal__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StartSignal_Response(metaclass=Metaclass_StartSignal_Response):
    """Message class 'StartSignal_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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


class Metaclass_StartSignal(type):
    """Metaclass of service 'StartSignal'."""

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
                'scene_srv.srv.StartSignal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__start_signal

            from scene_srv.srv import _start_signal
            if _start_signal.Metaclass_StartSignal_Request._TYPE_SUPPORT is None:
                _start_signal.Metaclass_StartSignal_Request.__import_type_support__()
            if _start_signal.Metaclass_StartSignal_Response._TYPE_SUPPORT is None:
                _start_signal.Metaclass_StartSignal_Response.__import_type_support__()


class StartSignal(metaclass=Metaclass_StartSignal):
    from scene_srv.srv._start_signal import StartSignal_Request as Request
    from scene_srv.srv._start_signal import StartSignal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
