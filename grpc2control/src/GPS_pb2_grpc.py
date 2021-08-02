# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

import GPS_pb2 as GPS__pb2


class GPSStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.gps = channel.unary_unary(
                '/GPS.GPS/gps',
                request_serializer=GPS__pb2.GPS_msg.SerializeToString,
                response_deserializer=GPS__pb2.Log.FromString,
                )


class GPSServicer(object):
    """Missing associated documentation comment in .proto file."""

    def gps(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_GPSServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'gps': grpc.unary_unary_rpc_method_handler(
                    servicer.gps,
                    request_deserializer=GPS__pb2.GPS_msg.FromString,
                    response_serializer=GPS__pb2.Log.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'GPS.GPS', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class GPS(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def gps(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/GPS.GPS/gps',
            GPS__pb2.GPS_msg.SerializeToString,
            GPS__pb2.Log.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)