"""The Python implementation of the GRPC adm2vis.Visual coreserver for data test."""


from concurrent import futures
import logging

import grpc

import task_pb2
import task_pb2_grpc
import adm2vis_pb2
import adm2vis_pb2_grpc
import adm2ctrl_pb2
import adm2ctrl_pb2_grpc

import time
import threading

import numpy as np
import random as rd


'''
Define global constant data or variables
'''
TIME_INTERVAL = 1.0                 # time interval for sending in seconds
TD_SEND_ADDR = 'localhost:20200'    # sending addr of task decomposition module, "td" in short
VIS_RCV_ADDR = 'localhost:20202'    # receiving addr of visual module, "vis" in short
VIS_SEND_ADDR = 'localhost:20204'   # sending addr of ...
CTRL_RCV_ADDR = 'localhost:20206'   # receiving addr of control module, "ctrl" in short
CTRL_SEND_ADDR = 'localhost:20208'  # sending addr of ...
COUNT_MAX = 20                      # max circular number in ADM

send_taskDecomp_flag = False        # send task decomposition info, pulse signal
send_calc_flag = False              # send calculation info (points and hvalues), step signal
send_gantt_flag = False             # send gantt, pulse signal
send_wta_flag = False               # send wta, pulse signal
send_tasks_flag = False             # send tasks to execute, pulse signal
send_attack_flag = False            # send attack pairs to execute, pulse signal

tddata_singletaskinfo = None        # task decomp data for task decomposition interface
visdata_plans = None                # visual data for point chart
visdata_hvalues = None              # visual data for line chart
visdata_tasks = None                # visual data for gantt chart
visdata_strategies = None           # visual data for wta table
ctrldata_tasks = None               # control data for task execution
ctrldata_attack = None              # control data for attack pairs execution


'''
Sending data/info: point, line, gantt, and wta table
'''
def SendTaskDecomp(singletaskinfo_data):    # send task decomposition info to visualization module
    global TD_SEND_ADDR
    with grpc.insecure_channel(TD_SEND_ADDR) as channel:
        stub = task_pb2_grpc.gRPCStub(channel)
        taskDecomp_info = stub.SendTaskInfo(task_pb2.RequestTaskInfo(singletaskinfo=singletaskinfo_data))
    print("coreserver received: " + str(taskDecomp_info.code))

def SendVisPoint(plans_data):               # send points to visualization module
    global VIS_SEND_ADDR
    with grpc.insecure_channel(VIS_SEND_ADDR) as channel:
        stub = adm2vis_pb2_grpc.VisualStub(channel)
        point_info = stub.VisPoints(adm2vis_pb2.PointsRequest(plans=plans_data))
    print("coreserver received: " + str(point_info.points_flag))

def SendVisLine(hvalues_data):              # send line to visualization module
    global VIS_SEND_ADDR
    with grpc.insecure_channel(VIS_SEND_ADDR) as channel:
        stub = adm2vis_pb2_grpc.VisualStub(channel)
        line_info = stub.VisLine(adm2vis_pb2.LineRequest(hvalues=hvalues_data))
    print("coreserver received: " + str(line_info.line_flag))

def SendVisGantt(tasks_data):               # send gantt to visualization module
    global VIS_SEND_ADDR
    with grpc.insecure_channel(VIS_SEND_ADDR) as channel:
        stub = adm2vis_pb2_grpc.VisualStub(channel)
        gantt_info = stub.VisGantt(adm2vis_pb2.GanttRequest(tasks=tasks_data))
    print("coreserver received: " + str(gantt_info.gantt_flag))

def SendVisWTA(strategies_data):            # send wta table to visualization module
    global VIS_SEND_ADDR
    with grpc.insecure_channel(VIS_SEND_ADDR) as channel:
        stub = adm2vis_pb2_grpc.VisualStub(channel)
        wta_info = stub.VisWTA(adm2vis_pb2.WTARequest(strategies=strategies_data))
    print("coreserver received: " + str(wta_info.wta_flag))

def SendCtrlTasks(tasks_data):
    global CTRL_SEND_ADDR
    with grpc.insecure_channel(CTRL_SEND_ADDR) as channel:
        stub = adm2ctrl_pb2_grpc.DtoCStub(channel)
        tasks_info = stub.ExecTasks(adm2ctrl_pb2.TasksRequest(tasks=tasks_data))
    print("coreserver received: " + str(tasks_info.tasks_flag))

def SendCtrlAttack(attack_pairs_data):
    global CTRL_SEND_ADDR
    with grpc.insecure_channel(CTRL_SEND_ADDR) as channel:
        stub = adm2ctrl_pb2_grpc.DtoCStub(channel)
        attack_info = stub.ExecAttack(adm2ctrl_pb2.AttackRequest(attack_pairs=attack_pairs_data))
    print("coreserver received: " + str(attack_info.attack_flag))


'''
Sending function: send data/info to visulization, task decomposition, ...
'''
def Sending():
    global TIME_INTERVAL, COUNT_MAX                                                                     # constant data
    global send_taskDecomp_flag, send_calc_flag, send_gantt_flag, send_wta_flag, \
        send_tasks_flag, send_attack_flag
    global tddata_singletaskinfo, visdata_plans, visdata_hvalues, visdata_tasks, visdata_strategies, \
         ctrldata_tasks, ctrldata_attack
    while True:
        if send_taskDecomp_flag:            # once
            SendTaskDecomp(tddata_singletaskinfo)
            send_taskDecomp_flag = False
        if send_calc_flag:                  # circular
            SendVisPoint(visdata_plans)
            SendVisLine(visdata_hvalues)
            time.sleep(TIME_INTERVAL)
        if send_gantt_flag:                 # several times
            SendVisGantt(visdata_tasks)
            send_gantt_flag = False
        if send_wta_flag:                   # several times
            SendVisWTA(visdata_strategies)
            send_wta_flag = False
        if send_tasks_flag:
            SendCtrlTasks(ctrldata_tasks)   # several times
            send_tasks_flag = False
        if send_attack_flag:
            SendCtrlAttack(ctrldata_attack) # several times
            send_attack_flag = False


'''
Our main fucntion: aided decision making = planning + assignment
(making up data for test)
'''
def ADM():     # model our ADM algorithm processing and storing data
    global TIME_INTERVAL, COUNT_MAX
    global send_taskDecomp_flag, send_calc_flag, send_gantt_flag, send_wta_flag, \
        send_tasks_flag, send_attack_flag
    global tddata_singletaskinfo, visdata_plans, visdata_hvalues, visdata_tasks, visdata_strategies, \
        ctrldata_tasks, ctrldata_attack
    # info in task decomposition interface
    singletaskinfo = []
    STIrequest = task_pb2.RequestTaskInfo()
    for i in range (4):
        single_task = STIrequest.singletaskinfo.add()
        single_task.taskOrder = i + 1
        single_task.taskType = i % 3 + 1
        single_task.pos.posx = rd.random() * 10
        single_task.pos.posy = rd.random() * 10
        single_task.pos.posz = 0.
        if i == 0:
            single_task.m_fromid[:] = [-1]
            single_task.m_toid[:] = [2, 3]
        elif i == 1 or i == 2:
            single_task.m_fromid[:] = [1]
            single_task.m_toid[:] = [4]
        else:
            single_task.m_fromid[:] = [2, 3]
            single_task.m_toid[:] = [-1]
        single_task.agent_us[:] = [i + 1, i + 2]
        single_task.agent_enemy[:] = [-1]
        single_task.taskName = "T" + str(i + 1)
        single_task.sendTime = time.time()
        singletaskinfo.append(single_task)
    tddata_singletaskinfo = singletaskinfo
    send_taskDecomp_flag = True

    count = 0
    while True:
        # assignment of plans in point chart
        plans = []
        pointsrequest = adm2vis_pb2.PointsRequest()
        for i in range(5):
            plan = pointsrequest.plans.add()
            plan.duration = rd.random() * 10
            plan.succ_rate = rd.random()
            plans.append(plan)
        visdata_plans = plans
        # assignment of hvalues in line chart
        hvalues = np.random.rand(count) * 10
        visdata_hvalues = hvalues
        if count > 0 and count < COUNT_MAX: # model the calculation is running
            send_calc_flag = True
        else:
            send_calc_flag = False
        # assignment of tasks in gantt chart
        tasks = []
        tasksrequest = adm2vis_pb2.GanttRequest()
        for i in range(4):
            task = tasksrequest.tasks.add()
            task.is_ended = 0
            if i == 2:
                task.is_ended = 1
            task.type = 1
            task.agent_us[:] = [1, 2, 4]
            task.agent_enemy[:] = [-1]
            task.start_time = rd.random() * 5
            task.duration = 2.5 + rd.random() * 10
            tasks.append(task)
        visdata_tasks = tasks
        if count == COUNT_MAX or count == COUNT_MAX + 5 or count == COUNT_MAX + 10:    # model the calculation is stopped or changed
            send_gantt_flag = True
        # assignment of strategies in wta table
        strategies = []
        strategiesrequest = adm2vis_pb2.WTARequest()
        for i in range(3):
            strategy = strategiesrequest.strategies.add()
            strategy.description = "high succ_rate"
            strategy.agent_us[:] = [1, 2, 3, 4]
            strategy.agent_enemy[:] = [1, 2, 3]
            strategy.attack_pair = "0103*0202*0301*0403*"
            strategy.agent_us_pre[:] = [1, 2]
            strategy.agent_enemy_pre[:] = [3]
            strategy.attack_pair_pre = "0103*0203*"
            strategy.win_rate = 0.5 + rd.random() / 2
            strategy.duration = 2.5 + rd.random() * 10
            strategies.append(strategy)
        visdata_strategies = strategies
        if count == COUNT_MAX or count == COUNT_MAX + 5 or count == COUNT_MAX + 10:    # model the calculation is stopped or changed
            send_wta_flag = True
        # assignment of tasks to execute
        tasks = []
        tasksrequest = adm2ctrl_pb2.TasksRequest()
        for i in range(4):
            task = tasksrequest.tasks.add()
            task.order = 1
            task.type = 1
            task.pos.posx = rd.random() * 10
            task.pos.posy = rd.random() * 10
            task.pos.posz = 0.
            task.agent_us[:] = [1, 2, 3]
            task.agent_enemy[:] = [-1]
            task.start_time = rd.random() * 5
            task.duration = 2.5 + rd.random() * 10
            tasks.append(task)
        ctrldata_tasks = tasks
        if count == COUNT_MAX + 10:
            send_tasks_flag = True
        # assignment of attack pairs to execute
        ctrldata_attack = ["0103*0202*0301*0403*", "0103*0203*"]
        if count == COUNT_MAX + 10:
            send_attack_flag = True
        # update count and wait
        count += 1
        time.sleep(TIME_INTERVAL/1.5)   # model asynchronous calculating-sending


'''
Receiving functions: receive data/info from decomposition/visulization module
'''
'''
class Visual(adm2vis_pb2_grpc.VisualServicer):  # the key component of data receiving and sending

    def TestArray(self, request, context):
        global data_rcv, data_flag
        data_rcv = request.values
        data_flag = True
        print("Visual core server received test info: " + str(data_rcv))
        return adm2vis_pb2.ArrayReply(flag="test array received")

def ReceiveFromVisual():    # receive from visual module
    global VIS_RCV_ADDR
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    adm2vis_pb2_grpc.add_VisualServicer_to_server(Visual(), server)
    server.add_insecure_port(VIS_RCV_ADDR)
    server.start()
    server.wait_for_termination()
'''


'''
Thread management
'''
threads = []
# thd_rcv = threading.Thread(target=ReceiveFromVisual)
# threads.append(thd_rcv)
thd_adm = threading.Thread(target=ADM)
threads.append(thd_adm)
thd_send = threading.Thread(target=Sending)
threads.append(thd_send)


if __name__ == '__main__':
    logging.basicConfig()

    for thd in threads:
        thd.start()
