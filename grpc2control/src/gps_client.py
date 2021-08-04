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


def gpsCallback(msg):
    logging.basicConfig()
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = pb2_grpc.GPSStub(channel)
        gps_msg = pb2.GPS_msg(Code='01')
        gps_msg.x = msg.pose.pose.position.x
        gps_msg.y = msg.pose.pose.position.y
        gps_msg.z = msg.twist.twist.angular.z
        gps_msg.v_x = msg.twist.twist.linear.x
        gps_msg.v_y = msg.twist.twist.linear.y
        response = stub.gps(gps_msg)
        print(response.data)


def gps_subscriber():
    rospy.init_node('gps_subscriber', anonymous=True)
    rospy.Subscriber('/car5/gps', Odometry, gpsCallback)
    rospy.spin()


if __name__ == '__main__':
    gps_subscriber()