"""Runtime protobuf definitions for the coordinated dual-arm trajectory RPC."""

from google.protobuf import descriptor_pb2
from google.protobuf import descriptor_pool
from google.protobuf.message_factory import GetMessageClass

from common.aimdk.protocol.common import joint_pb2  # noqa: F401


_TYPE_MESSAGE = descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE
_LABEL_REPEATED = descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED


def _field(message, name, number, field_type, label=None, type_name=None):
    field = message.field.add()
    field.name = name
    field.number = number
    field.type = field_type
    if label is not None:
        field.label = label
    if type_name is not None:
        field.type_name = type_name


_file = descriptor_pb2.FileDescriptorProto()
_file.name = "aimdk/protocol/sim/bimanual_trajectory.proto"
_file.package = "aimdk.protocol"
_file.syntax = "proto3"
_file.dependency.append("aimdk/protocol/common/joint.proto")

_point = _file.message_type.add()
_point.name = "BimanualTrajectoryPoint"
_field(_point, "left_joint", 1, _TYPE_MESSAGE, _LABEL_REPEATED, ".aimdk.protocol.JointCommand")
_field(_point, "right_joint", 2, _TYPE_MESSAGE, _LABEL_REPEATED, ".aimdk.protocol.JointCommand")

_request = _file.message_type.add()
_request.name = "BimanualTrajectoryReq"
_field(
    _request,
    "point",
    1,
    _TYPE_MESSAGE,
    _LABEL_REPEATED,
    ".aimdk.protocol.BimanualTrajectoryPoint",
)
_field(_request, "left_gripper_command", 2, descriptor_pb2.FieldDescriptorProto.TYPE_STRING)
_field(_request, "right_gripper_command", 3, descriptor_pb2.FieldDescriptorProto.TYPE_STRING)
_field(_request, "left_opened_width", 4, descriptor_pb2.FieldDescriptorProto.TYPE_DOUBLE)
_field(_request, "right_opened_width", 5, descriptor_pb2.FieldDescriptorProto.TYPE_DOUBLE)

_response = _file.message_type.add()
_response.name = "BimanualTrajectoryRsp"
_field(_response, "success", 1, descriptor_pb2.FieldDescriptorProto.TYPE_BOOL)
_field(_response, "message", 2, descriptor_pb2.FieldDescriptorProto.TYPE_STRING)

_service = _file.service.add()
_service.name = "BimanualTrajectoryService"
_method = _service.method.add()
_method.name = "move"
_method.input_type = ".aimdk.protocol.BimanualTrajectoryReq"
_method.output_type = ".aimdk.protocol.BimanualTrajectoryRsp"
_method = _service.method.add()
_method.name = "set_gripper"
_method.input_type = ".aimdk.protocol.BimanualTrajectoryReq"
_method.output_type = ".aimdk.protocol.BimanualTrajectoryRsp"

DESCRIPTOR = descriptor_pool.Default().AddSerializedFile(_file.SerializeToString())
BimanualTrajectoryPoint = GetMessageClass(DESCRIPTOR.message_types_by_name["BimanualTrajectoryPoint"])
BimanualTrajectoryReq = GetMessageClass(DESCRIPTOR.message_types_by_name["BimanualTrajectoryReq"])
BimanualTrajectoryRsp = GetMessageClass(DESCRIPTOR.message_types_by_name["BimanualTrajectoryRsp"])
