#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from communication.msg import TaskPos, Task, TasksRequest, AttackRequest

from concurrent import futures
import logging

import grpc

import adm2ctrl_pb2
import adm2ctrl_pb2_grpc
import threading

CTRL_RCV_ADDR = '192.168.43.78:20208'

task_ready_flag = False
task_msg = TasksRequest()

attack_ready_flag = False
attack_msg = AttackRequest()

def pub_task_info():
    pub = rospy.Publisher('TasksRequest', TasksRequest, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if not task_ready_flag:
            rate.sleep()
        pub.publish(task_msg)
        rate.sleep()

def pub_attack_info():
    pub = rospy.Publisher('AttackRequest', AttackRequest, queue_size=1)    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if not task_ready_flag:
            rate.sleep()
        pub.publish(attack_msg)
        rate.sleep()


class DtoC(adm2ctrl_pb2_grpc.DtoCServicer):

    def ExecTasks(self, request, context):
        print('enter ExecTasks')
        pub = rospy.Publisher('TasksRequest', TasksRequest, queue_size=1)
        global task_msg, task_ready_flag

        task_msg.init_timestamp = request.init_timestamp
        for task in request.tasks:
            t = Task()
            t.order = task.order
            t.type = task.type
            t.pos.posx = task.pos.posx
            t.pos.posy = task.pos.posy
            t.pos.posz = task.pos.posz
            for agent in task.agent_us:
                a = agent
                t.agent_us.append(a)
            for agent in task.agent_enemy:
                a = agent
                t.agent_us.append(a)
            t.start_time = task.start_time
            t.duration = task.duration
            task_msg.tasks.append(t)
        
        task_ready_flag = True
        pub.publish(task_msg)

        print("control server received: " + str(request.tasks))
        return adm2ctrl_pb2.TasksReply(tasks_flag="tasks received")

    def ExecAttack(self, request, context):
        print('enter ExecTasks')
        pub = rospy.Publisher('AttackRequest', AttackRequest, queue_size=1)
        global attack_msg, attack_ready_flag
        attack_msg = AttackRequest()
        for attack_pair in request.attack_pairs:
            a = str(attack_pair)
            attack_msg.attack_pairs.append(a)

        pub.publish(attack_msg)

        print("control server received: " + str(request.attack_pairs))
        attack_ready_flag = True
        
        return adm2ctrl_pb2.AttackReply(attack_flag="attack pairs received")


def DtoCServer():
    global CTRL_RCV_ADDR
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    adm2ctrl_pb2_grpc.add_DtoCServicer_to_server(DtoC(), server)
    server.add_insecure_port(CTRL_RCV_ADDR)
    server.start()
    print("Waiting for request")
    server.wait_for_termination()


threads = []
thd_ctrl = threading.Thread(target=DtoCServer)
threads.append(thd_ctrl)
thd_ros = threading.Thread(target=pub_task_info)
threads.append(thd_ros)


if __name__ == '__main__':
    logging.basicConfig()
    rospy.init_node('DtoCServer', anonymous=True)

    for thd in threads:
        thd.start()