#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
发送小车轨迹程序.
ros2grpc
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

car_id_list = [1,2,7]
n_car = 3

# 赵工IP
IP_CORE= '192.168.1.162'
PORT_ASK_DATA = '19330'

def trajCallback(msg, i_car):
    logging.basicConfig()
    with grpc.insecure_channel(IP_CORE+':'+PORT_ASK_DATA) as channel:
        stub = pb2_grpc.DataTransfServiceStub(channel)
        
        # 通过智能体编号筛选属于它的编号
        trajectory = pb2.GuiJiInfo(zntCode=str(car_id_list[i_car]))
        # print('publish ', car_id_list[i_car], 'traj')
        # print('n_points: ', len(msg.roadpoints))
        rospy.loginfo('receive car '+ str(car_id_list[i_car])+ 'trajectory. Length '+ str(len(msg.roadpoints)))

        for point in msg.roadpoints:
            roadpoint = trajectory.ghGuiJi.add()
            roadpoint.code = point.code
            roadpoint.x = point.x
            roadpoint.y = point.y
            roadpoint.v = point.v
            roadpoint.a = point.a
            roadpoint.yaw = point.yaw
            roadpoint.kappa = point.kappa
            
        response = stub.GuiJIGuiHua(trajectory)
        print('receive car '+ str(car_id_list[i_car])+ 'trajectory. Return ' + response.msg)



def traj_subscriber():
    rospy.init_node('traj_subscriber', anonymous=True)
    for i_car in range(n_car):
        rospy.Subscriber('/car'+str(car_id_list[i_car])+'/local_trajectory', Trajectory, trajCallback, i_car)
    rospy.spin()


if __name__ == '__main__':
    traj_subscriber()

