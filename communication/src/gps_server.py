#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from nav_msgs.msg import Odometry

from time import sleep
import grpc
import logging
from concurrent import futures

import data_transf_pb2 as pb2
import data_transf_pb2_grpc as pb2_grpc
import sys

'''
与中心节点传输数据所用的IP与端口号
'''
IP_CORE = '166.111.50.163'  # FIXME: 修改为运行数据交互节点的IP
IP_SITU = '183.173.115.75'  # FIXME: 修改为运行态势节点的主机IP
PORT_ASK_DATA = '19330'
PORT_CORE2SITU = '40010'  # FIXME: 修改为本机预备接受回传数据的端口号



class GPS(pb2_grpc.DataTransfServiceServicer):
    def ZNTStatusReceive(self, request, context):
        print("Receive agent", request.zntCode)

        pub = rospy.Publisher('/car1/gps', Odometry, queue_size=1)
        # rate = rospy.Rate(10)

        gps_msg = Odometry()
        print(request.hPosition, request.vPosition, request.hxAngle)
        gps_msg.pose.pose.position.x = request.hPosition
        gps_msg.pose.pose.position.y = request.vPosition
        gps_msg.twist.twist.angular.z = request.hxAngle

        pub.publish(gps_msg)
        # rate.sleep()

        response = pb2.BaseRespInfo(
            code='200',
            msg="SUCCESS",
            # serverTime= # FIXME: 时间戳应该写啥
        )
        return response


def Ask_Data():
    '''
    向数据交互模块申请发送智能体状态信息
    '''
    print('Asking for data...')
    with grpc.insecure_channel(IP_CORE+':'+PORT_ASK_DATA) as channel:
        stub = pb2_grpc.DataTransfServiceStub(channel)
        msg = pb2.SubscribeInfo(
            moduleCode='XT',
            operType='DY',
            moduleHost=IP_SITU,
            modulePort=int(PORT_CORE2SITU),
            dataCodes='ZNTZT',
            # sendTime=0 # FIXME: sendTime写啥？
        )
        response = stub.SubscribeData(msg)
        if response.code == '200':
            print('Success')
        else:
            logging.ERROR(response.msg)
            sys.exit()


def serve():
    rospy.init_node('talker', anonymous=True)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    
    pb2_grpc.add_DataTransfServiceServicer_to_server(
        GPS(), server)
    server.add_insecure_port(IP_SITU+':'+PORT_CORE2SITU)
    server.start()
    print("Waiting for request")
    server.wait_for_termination()


if __name__ == '__main__':
    logging.basicConfig()
    Ask_Data()
    serve()