"""gRPC bindings for the coordinated dual-arm trajectory RPC."""

import grpc

from common.aimdk.protocol.sim import bimanual_trajectory_pb2


class BimanualTrajectoryServiceStub:
    def __init__(self, channel):
        self.move = channel.unary_unary(
            "/aimdk.protocol.BimanualTrajectoryService/move",
            request_serializer=bimanual_trajectory_pb2.BimanualTrajectoryReq.SerializeToString,
            response_deserializer=bimanual_trajectory_pb2.BimanualTrajectoryRsp.FromString,
        )
        self.set_gripper = channel.unary_unary(
            "/aimdk.protocol.BimanualTrajectoryService/set_gripper",
            request_serializer=bimanual_trajectory_pb2.BimanualTrajectoryReq.SerializeToString,
            response_deserializer=bimanual_trajectory_pb2.BimanualTrajectoryRsp.FromString,
        )


class BimanualTrajectoryServiceServicer:
    def move(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")

    def set_gripper(self, request, context):
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details("Method not implemented!")
        raise NotImplementedError("Method not implemented!")


def add_BimanualTrajectoryServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
        "move": grpc.unary_unary_rpc_method_handler(
            servicer.move,
            request_deserializer=bimanual_trajectory_pb2.BimanualTrajectoryReq.FromString,
            response_serializer=bimanual_trajectory_pb2.BimanualTrajectoryRsp.SerializeToString,
        ),
        "set_gripper": grpc.unary_unary_rpc_method_handler(
            servicer.set_gripper,
            request_deserializer=bimanual_trajectory_pb2.BimanualTrajectoryReq.FromString,
            response_serializer=bimanual_trajectory_pb2.BimanualTrajectoryRsp.SerializeToString,
        ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
        "aimdk.protocol.BimanualTrajectoryService", rpc_method_handlers
    )
    server.add_generic_rpc_handlers((generic_handler,))
