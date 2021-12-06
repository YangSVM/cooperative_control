#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''借鉴态势信息
'''

from threading import Thread, Lock, Timer

import rospy
from nav_msgs.msg import Odometry

import grpc
import logging
from concurrent import futures

import data_transf_pb2 
import data_transf_pb2_grpc
import sys

IP_CORE = '166.111.188.213'  # FIXME: 修改为运行数据交互节点的IP
IP_SITU = '183.173.66.49'  # FIXME: 修改为运行态势节点的主机IP
PORT_ASK_DATA = '19330'
PORT_CORE2SITU = '40000'  # FIXME: 修改为本机预备接受回传数据的端口号


class DataTransfServicer(data_transf_pb2_grpc.DataTransfServiceServicer):
    def ZNTStatusReceive(self, request, context):

        print("Receive agent", request.zntCode)
        # agent_list.update(
        #     request.zntCode, request.zntPosition, request.hxAngle)  # FIXME: 向工程师确认编码格式，朝向的角度单位

        response = data_transf_pb2.BaseRespInfo(
            code='200',
            msg="SUCCESS",
            # serverTime= # FIXME: 时间戳应该写啥
        )
        return response



def Rec_from_Core():
    '''
    接收数据交互模块所发来的信息，具体方法实现在类DataTransfServicer()中
    '''
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    data_transf_pb2_grpc.add_DataTransfServiceServicer_to_server(
        DataTransfServicer(), server)
    server.add_insecure_port(IP_SITU+':'+PORT_CORE2SITU)
    server.start()
    server.wait_for_termination()


def Ask_Data():
    '''
    向数据交互模块申请发送智能体状态信息
    '''
    print('Asking for data...')
    with grpc.insecure_channel(IP_CORE+':'+PORT_ASK_DATA) as channel:
        stub = data_transf_pb2_grpc.DataTransfServiceStub(channel)
        msg = data_transf_pb2.SubscribeInfo(
            moduleCode='TS',
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

if __name__ == '__main__':
    logging.basicConfig()
    Ask_Data()
    rospy.init_node('formation_gps_request', anonymous=True)

    thd_rec = Thread(target=Rec_from_Core)
    thd_rec.start()