#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import math
import numpy as np
from trajectory_tracking.msg import RoadPoint
from trajectory_tracking.msg import Trajectory

from time import sleep
import grpc
import logging
from concurrent import futures

import Traj_Plan_pb2 as pb2
import Traj_Plan_pb2_grpc as pb2_grpc


def trajCallback(msg):
    logging.basicConfig()
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = pb2_grpc.Traj_PlanStub(channel)
        trajectory = pb2.Trajectory(Code='1')
        for point in msg.roadpoints:
            roadpoint = trajectory.RoadPoints.add()
            roadpoint.x = point.x
            roadpoint.y = point.y
            roadpoint.v = point.v
            roadpoint.a = point.a
            roadpoint.yaw = point.yaw
            roadpoint.kappa = point.kappa
        responce = stub.TrajPlan(trajectory)
        print(responce.Log)


def traj_subscriber():
    rospy.init_node('traj_subscriber', anonymous=True)
    rospy.Subscriber('/car5/local_trajectory', Trajectory, trajCallback)
    rospy.spin()


if __name__ == '__main__':
    traj_subscriber()

