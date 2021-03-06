# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: robot_data.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='robot_data.proto',
  package='',
  serialized_pb=_b('\n\x10robot_data.proto\"\xb5\x01\n\tRobotData\x12\x0e\n\x06s0_pos\x18\x01 \x02(\x05\x12\x0e\n\x06s1_pos\x18\x02 \x02(\x05\x12\x11\n\x06sonarf\x18\x03 \x02(\x03:\x01\x30\x12\x11\n\x06sonarb\x18\x04 \x02(\x03:\x01\x30\x12/\n\x0bled_pattern\x18\x05 \x02(\x0e\x32\x15.RobotData.LedPattern:\x03OFF\"1\n\nLedPattern\x12\x07\n\x03OFF\x10\x00\x12\r\n\tCOLORWIPE\x10\x01\x12\x0b\n\x07RAINBOW\x10\x02')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_ROBOTDATA_LEDPATTERN = _descriptor.EnumDescriptor(
  name='LedPattern',
  full_name='RobotData.LedPattern',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OFF', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLORWIPE', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RAINBOW', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=153,
  serialized_end=202,
)
_sym_db.RegisterEnumDescriptor(_ROBOTDATA_LEDPATTERN)


_ROBOTDATA = _descriptor.Descriptor(
  name='RobotData',
  full_name='RobotData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='s0_pos', full_name='RobotData.s0_pos', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s1_pos', full_name='RobotData.s1_pos', index=1,
      number=2, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sonarf', full_name='RobotData.sonarf', index=2,
      number=3, type=3, cpp_type=2, label=2,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sonarb', full_name='RobotData.sonarb', index=3,
      number=4, type=3, cpp_type=2, label=2,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='led_pattern', full_name='RobotData.led_pattern', index=4,
      number=5, type=14, cpp_type=8, label=2,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _ROBOTDATA_LEDPATTERN,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=21,
  serialized_end=202,
)

_ROBOTDATA.fields_by_name['led_pattern'].enum_type = _ROBOTDATA_LEDPATTERN
_ROBOTDATA_LEDPATTERN.containing_type = _ROBOTDATA
DESCRIPTOR.message_types_by_name['RobotData'] = _ROBOTDATA

RobotData = _reflection.GeneratedProtocolMessageType('RobotData', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTDATA,
  __module__ = 'robot_data_pb2'
  # @@protoc_insertion_point(class_scope:RobotData)
  ))
_sym_db.RegisterMessage(RobotData)


# @@protoc_insertion_point(module_scope)
