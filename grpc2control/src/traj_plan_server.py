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


class Traj_Plan(pb2_grpc.Traj_PlanServicer):
    def TrajPlan(self, request, context):
        pub = rospy.Publisher('trajectory', Trajectory, queue_size=1)

        rate = rospy.Rate(10)

        local_trajectory = Trajectory()
        for point in request.RoadPoints:
            road_point = RoadPoint()
            road_point.x = point.x  # x
            road_point.y = point.y  # y
            road_point.v = point.v  # v
            road_point.a = point.a  # a
            road_point.yaw = point.yaw  # theta heading
            road_point.kappa = point.kappa  # kappa
            local_trajectory.roadpoints.append(road_point)
        pub.publish(local_trajectory)
        rate.sleep()

        return pb2.Log(data="Success")


def serve():
    rospy.init_node('talker', anonymous=True)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    pb2_grpc.add_Traj_PlanServicer_to_server(
        Traj_Plan(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    print("Waiting for request")
    server.wait_for_termination()


if __name__ == '__main__':
    logging.basicConfig()
    serve()
        



