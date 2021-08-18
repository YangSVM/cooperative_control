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

import data_transf_pb2 as pb2
import data_transf_pb2_grpc as pb2_grpc

IP_CORE = '166.111.50.163'  # FIXME: 修改为运行数据交互节点的IP
IP_SITU = '192.168.8.108'  # FIXME: 修改为运行态势节点的主机IP
PORT_ASK_DATA = '19330'
PORT_CORE2SITU = '40010'  # FIXME: 修改为本机预备接受回传数据的端口号

def gpsCallback(msg):
    print('get gps information')
    logging.basicConfig()
    with grpc.insecure_channel(IP_SITU+':'+PORT_CORE2SITU) as channel:
        print('connect success')
        stub = pb2_grpc.DataTransfServiceStub(channel)
        gps_msg = pb2.ZNTStatusInfo(zntCode='01')
        gps_msg.hPosition = msg.pose.pose.position.x
        gps_msg.vPosition = msg.pose.pose.position.y
        # gps_msg.z = msg.twist.twist.angular.z

        response = stub.ZNTStatusReceive(gps_msg)
        print(response.msg)


def gps_subscriber():
    rospy.init_node('gps_subscriber', anonymous=True)
    rospy.Subscriber('/car5/gps', Odometry, gpsCallback)
    rospy.spin()


if __name__ == '__main__':
    gps_subscriber()