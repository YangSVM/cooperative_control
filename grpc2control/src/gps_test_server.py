#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from time import sleep
import grpc
import logging
from concurrent import futures

import GPS_pb2 as pb2
import GPS_pb2_grpc as pb2_grpc


class GPS(pb2_grpc.GPSServicer):
    def gps(self, request, context):

        print("x: %f" % request.x)
        print("y: %f" % request.y)
        print("twist: %f" % request.z)
        print("v_x: %f" % request.v_x)
        print("v_y: %f" % request.v_y)

        return pb2.Log(data="Success")


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    pb2_grpc.add_GPSServicer_to_server(
        GPS(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    print("Waiting for request")
    server.wait_for_termination()


if __name__ == '__main__':
    logging.basicConfig()
    serve()