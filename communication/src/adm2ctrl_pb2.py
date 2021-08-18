# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: adm2ctrl.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='adm2ctrl.proto',
  package='adm2ctrl',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0e\x61\x64m2ctrl.proto\x12\x08\x61\x64m2ctrl\"\x94\x02\n\x0cTasksRequest\x12*\n\x05tasks\x18\x01 \x03(\x0b\x32\x1b.adm2ctrl.TasksRequest.Task\x1a\xd7\x01\n\x04Task\x12\r\n\x05order\x18\x01 \x01(\x05\x12\x0c\n\x04type\x18\x02 \x01(\x05\x12\x30\n\x03pos\x18\x03 \x01(\x0b\x32#.adm2ctrl.TasksRequest.Task.TaskPos\x12\x10\n\x08\x61gent_us\x18\x04 \x03(\x05\x12\x13\n\x0b\x61gent_enemy\x18\x05 \x03(\x05\x12\x12\n\nstart_time\x18\x06 \x01(\x01\x12\x10\n\x08\x64uration\x18\x07 \x01(\x01\x1a\x33\n\x07TaskPos\x12\x0c\n\x04posx\x18\x01 \x01(\x01\x12\x0c\n\x04posy\x18\x02 \x01(\x01\x12\x0c\n\x04posz\x18\x03 \x01(\x01\" \n\nTasksReply\x12\x12\n\ntasks_flag\x18\x01 \x01(\t\"%\n\rAttackRequest\x12\x14\n\x0c\x61ttack_pairs\x18\x01 \x03(\t\"\"\n\x0b\x41ttackReply\x12\x13\n\x0b\x61ttack_flag\x18\x01 \x01(\t\"\x84\x01\n\x0f\x45xecInfoRequest\x12\x32\n\x08tasks_ei\x18\x01 \x03(\x0b\x32 .adm2ctrl.ExecInfoRequest.TaskEI\x1a=\n\x06TaskEI\x12\r\n\x05order\x18\x01 \x01(\x05\x12\x0e\n\x06status\x18\x02 \x01(\x05\x12\x14\n\x0c\x64uration_upd\x18\x03 \x01(\x01\"&\n\rExecInfoReply\x12\x15\n\rexecinfo_flag\x18\x01 \x01(\t2\x83\x01\n\x04\x44toC\x12;\n\tExecTasks\x12\x16.adm2ctrl.TasksRequest\x1a\x14.adm2ctrl.TasksReply\"\x00\x12>\n\nExecAttack\x12\x17.adm2ctrl.AttackRequest\x1a\x15.adm2ctrl.AttackReply\"\x00\x32J\n\x04\x43toD\x12\x42\n\nFbExecInfo\x12\x19.adm2ctrl.ExecInfoRequest\x1a\x17.adm2ctrl.ExecInfoReply\"\x00\x62\x06proto3'
)




_TASKSREQUEST_TASK_TASKPOS = _descriptor.Descriptor(
  name='TaskPos',
  full_name='adm2ctrl.TasksRequest.Task.TaskPos',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='posx', full_name='adm2ctrl.TasksRequest.Task.TaskPos.posx', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='posy', full_name='adm2ctrl.TasksRequest.Task.TaskPos.posy', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='posz', full_name='adm2ctrl.TasksRequest.Task.TaskPos.posz', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=254,
  serialized_end=305,
)

_TASKSREQUEST_TASK = _descriptor.Descriptor(
  name='Task',
  full_name='adm2ctrl.TasksRequest.Task',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='order', full_name='adm2ctrl.TasksRequest.Task.order', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='type', full_name='adm2ctrl.TasksRequest.Task.type', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pos', full_name='adm2ctrl.TasksRequest.Task.pos', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='agent_us', full_name='adm2ctrl.TasksRequest.Task.agent_us', index=3,
      number=4, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='agent_enemy', full_name='adm2ctrl.TasksRequest.Task.agent_enemy', index=4,
      number=5, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='start_time', full_name='adm2ctrl.TasksRequest.Task.start_time', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='duration', full_name='adm2ctrl.TasksRequest.Task.duration', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_TASKSREQUEST_TASK_TASKPOS, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=90,
  serialized_end=305,
)

_TASKSREQUEST = _descriptor.Descriptor(
  name='TasksRequest',
  full_name='adm2ctrl.TasksRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='tasks', full_name='adm2ctrl.TasksRequest.tasks', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_TASKSREQUEST_TASK, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=29,
  serialized_end=305,
)


_TASKSREPLY = _descriptor.Descriptor(
  name='TasksReply',
  full_name='adm2ctrl.TasksReply',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='tasks_flag', full_name='adm2ctrl.TasksReply.tasks_flag', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=307,
  serialized_end=339,
)


_ATTACKREQUEST = _descriptor.Descriptor(
  name='AttackRequest',
  full_name='adm2ctrl.AttackRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='attack_pairs', full_name='adm2ctrl.AttackRequest.attack_pairs', index=0,
      number=1, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=341,
  serialized_end=378,
)


_ATTACKREPLY = _descriptor.Descriptor(
  name='AttackReply',
  full_name='adm2ctrl.AttackReply',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='attack_flag', full_name='adm2ctrl.AttackReply.attack_flag', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=380,
  serialized_end=414,
)


_EXECINFOREQUEST_TASKEI = _descriptor.Descriptor(
  name='TaskEI',
  full_name='adm2ctrl.ExecInfoRequest.TaskEI',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='order', full_name='adm2ctrl.ExecInfoRequest.TaskEI.order', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='status', full_name='adm2ctrl.ExecInfoRequest.TaskEI.status', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='duration_upd', full_name='adm2ctrl.ExecInfoRequest.TaskEI.duration_upd', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=488,
  serialized_end=549,
)

_EXECINFOREQUEST = _descriptor.Descriptor(
  name='ExecInfoRequest',
  full_name='adm2ctrl.ExecInfoRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='tasks_ei', full_name='adm2ctrl.ExecInfoRequest.tasks_ei', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_EXECINFOREQUEST_TASKEI, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=417,
  serialized_end=549,
)


_EXECINFOREPLY = _descriptor.Descriptor(
  name='ExecInfoReply',
  full_name='adm2ctrl.ExecInfoReply',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='execinfo_flag', full_name='adm2ctrl.ExecInfoReply.execinfo_flag', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=551,
  serialized_end=589,
)

_TASKSREQUEST_TASK_TASKPOS.containing_type = _TASKSREQUEST_TASK
_TASKSREQUEST_TASK.fields_by_name['pos'].message_type = _TASKSREQUEST_TASK_TASKPOS
_TASKSREQUEST_TASK.containing_type = _TASKSREQUEST
_TASKSREQUEST.fields_by_name['tasks'].message_type = _TASKSREQUEST_TASK
_EXECINFOREQUEST_TASKEI.containing_type = _EXECINFOREQUEST
_EXECINFOREQUEST.fields_by_name['tasks_ei'].message_type = _EXECINFOREQUEST_TASKEI
DESCRIPTOR.message_types_by_name['TasksRequest'] = _TASKSREQUEST
DESCRIPTOR.message_types_by_name['TasksReply'] = _TASKSREPLY
DESCRIPTOR.message_types_by_name['AttackRequest'] = _ATTACKREQUEST
DESCRIPTOR.message_types_by_name['AttackReply'] = _ATTACKREPLY
DESCRIPTOR.message_types_by_name['ExecInfoRequest'] = _EXECINFOREQUEST
DESCRIPTOR.message_types_by_name['ExecInfoReply'] = _EXECINFOREPLY
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TasksRequest = _reflection.GeneratedProtocolMessageType('TasksRequest', (_message.Message,), {

  'Task' : _reflection.GeneratedProtocolMessageType('Task', (_message.Message,), {

    'TaskPos' : _reflection.GeneratedProtocolMessageType('TaskPos', (_message.Message,), {
      'DESCRIPTOR' : _TASKSREQUEST_TASK_TASKPOS,
      '__module__' : 'adm2ctrl_pb2'
      # @@protoc_insertion_point(class_scope:adm2ctrl.TasksRequest.Task.TaskPos)
      })
    ,
    'DESCRIPTOR' : _TASKSREQUEST_TASK,
    '__module__' : 'adm2ctrl_pb2'
    # @@protoc_insertion_point(class_scope:adm2ctrl.TasksRequest.Task)
    })
  ,
  'DESCRIPTOR' : _TASKSREQUEST,
  '__module__' : 'adm2ctrl_pb2'
  # @@protoc_insertion_point(class_scope:adm2ctrl.TasksRequest)
  })
_sym_db.RegisterMessage(TasksRequest)
_sym_db.RegisterMessage(TasksRequest.Task)
_sym_db.RegisterMessage(TasksRequest.Task.TaskPos)

TasksReply = _reflection.GeneratedProtocolMessageType('TasksReply', (_message.Message,), {
  'DESCRIPTOR' : _TASKSREPLY,
  '__module__' : 'adm2ctrl_pb2'
  # @@protoc_insertion_point(class_scope:adm2ctrl.TasksReply)
  })
_sym_db.RegisterMessage(TasksReply)

AttackRequest = _reflection.GeneratedProtocolMessageType('AttackRequest', (_message.Message,), {
  'DESCRIPTOR' : _ATTACKREQUEST,
  '__module__' : 'adm2ctrl_pb2'
  # @@protoc_insertion_point(class_scope:adm2ctrl.AttackRequest)
  })
_sym_db.RegisterMessage(AttackRequest)

AttackReply = _reflection.GeneratedProtocolMessageType('AttackReply', (_message.Message,), {
  'DESCRIPTOR' : _ATTACKREPLY,
  '__module__' : 'adm2ctrl_pb2'
  # @@protoc_insertion_point(class_scope:adm2ctrl.AttackReply)
  })
_sym_db.RegisterMessage(AttackReply)

ExecInfoRequest = _reflection.GeneratedProtocolMessageType('ExecInfoRequest', (_message.Message,), {

  'TaskEI' : _reflection.GeneratedProtocolMessageType('TaskEI', (_message.Message,), {
    'DESCRIPTOR' : _EXECINFOREQUEST_TASKEI,
    '__module__' : 'adm2ctrl_pb2'
    # @@protoc_insertion_point(class_scope:adm2ctrl.ExecInfoRequest.TaskEI)
    })
  ,
  'DESCRIPTOR' : _EXECINFOREQUEST,
  '__module__' : 'adm2ctrl_pb2'
  # @@protoc_insertion_point(class_scope:adm2ctrl.ExecInfoRequest)
  })
_sym_db.RegisterMessage(ExecInfoRequest)
_sym_db.RegisterMessage(ExecInfoRequest.TaskEI)

ExecInfoReply = _reflection.GeneratedProtocolMessageType('ExecInfoReply', (_message.Message,), {
  'DESCRIPTOR' : _EXECINFOREPLY,
  '__module__' : 'adm2ctrl_pb2'
  # @@protoc_insertion_point(class_scope:adm2ctrl.ExecInfoReply)
  })
_sym_db.RegisterMessage(ExecInfoReply)



_DTOC = _descriptor.ServiceDescriptor(
  name='DtoC',
  full_name='adm2ctrl.DtoC',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=592,
  serialized_end=723,
  methods=[
  _descriptor.MethodDescriptor(
    name='ExecTasks',
    full_name='adm2ctrl.DtoC.ExecTasks',
    index=0,
    containing_service=None,
    input_type=_TASKSREQUEST,
    output_type=_TASKSREPLY,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='ExecAttack',
    full_name='adm2ctrl.DtoC.ExecAttack',
    index=1,
    containing_service=None,
    input_type=_ATTACKREQUEST,
    output_type=_ATTACKREPLY,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_DTOC)

DESCRIPTOR.services_by_name['DtoC'] = _DTOC


_CTOD = _descriptor.ServiceDescriptor(
  name='CtoD',
  full_name='adm2ctrl.CtoD',
  file=DESCRIPTOR,
  index=1,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=725,
  serialized_end=799,
  methods=[
  _descriptor.MethodDescriptor(
    name='FbExecInfo',
    full_name='adm2ctrl.CtoD.FbExecInfo',
    index=0,
    containing_service=None,
    input_type=_EXECINFOREQUEST,
    output_type=_EXECINFOREPLY,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_CTOD)

DESCRIPTOR.services_by_name['CtoD'] = _CTOD

# @@protoc_insertion_point(module_scope)