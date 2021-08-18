#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
发送小车轨迹程序
'''

import rospy
import math
import numpy as np
from trajectory_tracking.msg import RoadPoint
from trajectory_tracking.msg import Trajectory

from time import sleep
import grpc
import logging
from concurrent import futures

import data_transf_pb2 as pb2
import data_transf_pb2_grpc as pb2_grpc


def trajCallback(msg):
    logging.basicConfig()
    with grpc.insecure_channel('166.111.50.163:19330') as channel:
        stub = pb2_grpc.DataTransfServiceStub(channel)
        trajectory = pb2.GuiJiInfo(zntCode='ZNT002')
        for point in msg.roadpoints:
            roadpoint = trajectory.ghGuiJi.add()
            # roadpoint.code = point.code
            roadpoint.x = point.x
            roadpoint.y = point.y
            roadpoint.v = point.v
            roadpoint.a = point.a
            roadpoint.yaw = point.yaw
            roadpoint.kappa = point.kappa
            
        response = stub.GuiJIGuiHua(trajectory)
        print(response.msg)


def traj_subscriber():
    rospy.init_node('traj_subscriber', anonymous=True)
    rospy.Subscriber('/car5/local_trajectory', Trajectory, trajCallback)
    rospy.spin()


if __name__ == '__main__':
    traj_subscriber()

