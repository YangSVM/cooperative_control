#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''发送辅助决策信息
ros2grpc
'''
import rospy
from nav_msgs.msg import Odometry

from time import sleep
import grpc
import logging
from concurrent import futures

from communication.msg import ExecInfoRequest

import adm2ctrl_pb2 as pb2
import adm2ctrl_pb2_grpc as pb2_grpc

IP_CORE = '166.111.50.163'  # FIXME: 修改为运行数据交互节点的IP
IP_DECISION = '183.173.129.248'  # FIXME: 修改为运行决策节点的主机IP
PORT_CORE2DECISION = '20207'  # FIXME: 修改为本机预备接受回传数据的端口号



def task_info_callback(rosmsg):
    print('get gps information')
    n_tasks = len(rosmsg.tasks_ei)
    logging.basicConfig()
    with grpc.insecure_channel(IP_DECISION+':'+PORT_CORE2DECISION) as channel:
        print('connect success')
        stub = pb2_grpc.CtoDStub(channel)
        exec_info_request = pb2.ExecInfoRequest()
        for task_ei_ros in rosmsg.tasks_ei:
            task_ei = exec_info_request.tasks_ei.add()
            task_ei.order = task_ei_ros.order
            task_ei.status = task_ei_ros.status
            task_ei.duration_upd = task_ei_ros.duration_upd
        
        response = stub.FbExecInfo(exec_info_request)
        print(response)


def adm_replay():
    rospy.init_node('adm_replay')
    rospy.Subscriber('task_info', ExecInfoRequest, task_info_callback)
    # rospy.Subscriber('')
    rospy.spin()


if __name__ == '__main__':
    adm_replay()