#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry

from time import sleep
import grpc
import logging
from concurrent import futures

import GPS_pb2 as pb2
import GPS_pb2_grpc as pb2_grpc


class GPS(pb2_grpc.GPSServicer):
    def gps(self, request, context):
        pub = rospy.Publisher('/01/gps', Odometry, queue_size=1)

        rate = rospy.Rate(10)

        gps_msg = Odometry()
        gps_msg.pose.pose.position.x = request.x
        gps_msg.pose.pose.position.y = request.y
        gps_msg.twist.twist.angular.z = request.z
        gps_msg.twist.twist.linear.x = request.v_x
        gps_msg.twist.twist.linear.y = request.v_y
        pub.publish(gps_msg)
        rate.sleep()

        return pb2.Log(data="Success")


def serve():
    rospy.init_node('talker', anonymous=True)
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