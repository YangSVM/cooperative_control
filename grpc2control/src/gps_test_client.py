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


if __name__ == '__main__':
    logging.basicConfig()
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = pb2_grpc.GPSStub(channel)

        for time_stamp in range(60):
            gps_msg = pb2.GPS_msg(Code='01')
            gps_msg.x = time_stamp * 0.25 - 10.0
            gps_msg.y = 5.0
            gps_msg.z = 0.0
            gps_msg.v_x = 0.25
            gps_msg.v_y = 0
            response = stub.gps(gps_msg)
            print(response.data)
            gps_msg = pb2.GPS_msg(Code='02')
            gps_msg.x = time_stamp * 0.25 - 10.0
            gps_msg.y = 0.0
            gps_msg.z = 0.0
            gps_msg.v_x = 0.25
            gps_msg.v_y = 0
            response = stub.gps(gps_msg)
            print(response.data)
            gps_msg = pb2.GPS_msg(Code='03')
            gps_msg.x = time_stamp * 0.25 - 10.0
            gps_msg.y = -5.0
            gps_msg.z = 0.0
            gps_msg.v_x = 0.25
            gps_msg.v_y = 0.0
            response = stub.gps(gps_msg)
            print(response.data)
            gps_msg = pb2.GPS_msg(Code='04')
            gps_msg.x = -8.0
            gps_msg.y = 17.2
            gps_msg.z = 0.0
            gps_msg.v_x = 0.0
            gps_msg.v_y = 0.0
            response = stub.gps(gps_msg)
            print(response.data)
            gps_msg = pb2.GPS_msg(Code='05')
            gps_msg.x = -11.3
            gps_msg.y = 17.8
            gps_msg.z = 0.0
            gps_msg.v_x = 0.0
            gps_msg.v_y = 0.0
            response = stub.gps(gps_msg)
            print(response.data)
            gps_msg = pb2.GPS_msg(Code='06')
            gps_msg.x = -12.5
            gps_msg.y = 14.2
            gps_msg.z = 0.0
            gps_msg.v_x = 0.0
            gps_msg.v_y = 0.0
            response = stub.gps(gps_msg)
            print(response.data)
            sleep(1.0)