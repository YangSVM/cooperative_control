import grpc
import logging
from concurrent import futures

from threading import Thread, Lock, Timer
import cv2
import numpy as np
import sys

import numpy as np
from lib.agent import *
from lib.attack import *
from lib.defend import *
from lib.understanding import *
from lib.utils import *

import data_transf_pb2
import data_transf_pb2_grpc
import Situ_Visual_pb2
import Situ_Visual_pb2_grpc

'''
与中心节点传输数据所用的IP与端口号
'''
IP_CORE = '166.111.50.163'  # FIXME: 修改为运行数据交互节点的IP
IP_SITU = 'localhost'  # FIXME: 修改为运行态势节点的主机IP
PORT_ASK_DATA = '19330'
PORT_CORE2SITU = '40000'  # FIXME: 修改为本机预备接受回传数据的端口号

'''
与可视化传输数据所用的IP与端口号
'''
IP_VISUAL = 'localhost'  # FIXME: 修改为运行可视化节点的主机IP
PORT_SITU2VISUAL_AGTS = '50000'
PORT_SITU2VISUAL_ATK_HOST = '50001'
PORT_SITU2VISUAL_ATK_ENEMY = '50002'
PORT_SITU2VISUAL_DEF_HOST = '50003'
PORT_SITU2VISUAL_DEF_ENEMY = '50004'
PORT_SITU2VISUAL_PRED_SHORT = '50005'
PORT_SITU2VISUAL_PRED_LONG = '50006'

'''
与可视化传输所用的其它变量
'''
SEND_FLAG = False
Timer_Interval = 10  # 最小定时发送间隔，实际发送间隔=max(Timer_Interval, 接收core数据的间隔)
# 下述7个变量单位为1米
xlim_graph = 40  # 实际场地大小
ylim_graph = 45
xlim_msg = 62
ylim_msg = 49
x_offset = 6  # offset定义graph左侧和上侧距离msg的边缘的距离>=0
y_offset = 4
length = 1  # 定义态势数据栅格的精度
# 下述6个变量单位为态势数据栅格的1行/1列
row_range_graph = int(ylim_graph/length)  # 态势矩阵行数
column_range_graph = int(xlim_graph/length)
row_range_msg = int(ylim_msg/length)  # 传给可视化的态势消息行数
column_range_msg = int(xlim_msg/length)
row_offset = int(y_offset/length)
column_offset = int(x_offset/length)


lock = Lock()  # 全局线程锁


class RepeatingTimer(Timer):
    '''
    依靠多线程，实现定时循环器
    '''

    def run(self):
        while not self.finished.is_set():
            self.function(*self.args, **self.kwargs)
            self.finished.wait(self.interval)


class Agent_List():
    '''
    用于全局变量agent_list，储存态势相关数据，从而在多线程间通讯；
    #! 其中的time_stamp变量和各msg变量会被多个线程读写，调用前务必加锁
    '''

    def __init__(self):
        self.time_stamp = 0
        self.codeList = ['znt01']  # FIXME: code of D1,C1,I1 as string
        self.init_agents()
        self.init_graph()

        # update graph
        self.update_graph()
        lock.acquire()
        # update msg
        self.update_msg()
        lock.release()

    def init_agents(self):
        '''
        初始化数据库，在此处调整我方敌方目标的各初始属性
        '''
        Home_pos = np.array(
            [[0, 0], [0, 0], [0, 0]])
        Enemy_pos = np.array(
            [[17.019, 42.215], [13.638, 42.382], [17.129, 33.980]])
        Obstacle_pos = np.array(
            [[17.507, 41.284], [14.245, 41.553], [13.136, 38.480]])

        self.D1 = direct_fire(x0=Home_pos[0, 0], y0=Home_pos[0, 1], v0=0, type1="D", type2="M134", firing_rate=3000, m=0.023,
                              shoot_v=850, R_err=1, theta=139.9958, alpha=1.0/500, val=6, field=[6, 6], r_max=15, gamma=1, n=1, mob=[25, 6, 6])
        self.D1.Defpow = 5e6
        self.D1.side = 'home'

        self.C1 = indirect_fire(x0=Home_pos[1, 0], y0=Home_pos[1, 1], v0=0, type1="I", type2="Control_car", firing_rate=0, m_e=0, E0=0, R_exp=1,
                                r_min=0, r_prime=0, Ex=0, theta=140.7867, alpha=1.0/500, val=20, field=[5, 5], r_max=0, gamma=1, shoot_v=0, mob=[40, 8, 8])
        self.C1.Defpow = 3e6
        self.C1.side = 'home'

        self.I1 = indirect_fire(x0=Home_pos[2, 0], y0=Home_pos[2, 1], v0=0, type1="I", type2="PLZ05", firing_rate=8, m_e=0.04, E0=5e7, R_exp=3,
                                r_min=0, r_prime=0.1, Ex=1e-4, theta=138.528, alpha=1.0/200, val=10, field=[7, 7], r_max=25, gamma=1, shoot_v=900, mob=[25, 4, 3])
        self.I1.Defpow = 8e6
        self.I1.side = 'home'

        self.d1 = direct_fire(x0=Enemy_pos[0, 0], y0=Enemy_pos[0, 1], v0=0, type1="D", type2="M134", firing_rate=3000, m=0.023,
                              shoot_v=850, R_err=1, theta=319.9958, alpha=1.0/500, val=6, field=[6, 6], r_max=15, gamma=1, n=1, mob=[25, 6, 6])
        self.d1.Defpow = 5e6
        self.d1.side = 'enemy'

        self.c1 = indirect_fire(x0=Enemy_pos[1, 0], y0=Enemy_pos[1, 1], v0=0, type1="I", type2="Control_car", firing_rate=0, m_e=0, E0=0, R_exp=1,
                                r_min=0, r_prime=0, Ex=0, theta=320.7867, alpha=1.0/500, val=20, field=[5, 5], r_max=0, gamma=1, shoot_v=0, mob=[40, 8, 8])
        self.c1.Defpow = 3e6
        self.c1.side = 'enemy'

        self.i1 = indirect_fire(x0=Enemy_pos[2, 0], y0=Enemy_pos[2, 1], v0=0, type1="I", type2="PLZ05", firing_rate=8, m_e=0.04, E0=5e7, R_exp=3,
                                r_min=0, r_prime=0.1, Ex=1e-4, theta=318.528, alpha=1.0/200, val=10, field=[7, 7], r_max=25, gamma=1, shoot_v=900, mob=[25, 4, 3])
        self.i1.Defpow = 8e6
        self.i1.side = 'enemy'

        self.O1 = direct_fire(x0=Obstacle_pos[0, 0], y0=Obstacle_pos[0, 1], v0=0, type1="O", type2="", firing_rate=0, m=0,
                              shoot_v=0, R_err=1, theta=0, alpha=0, val=0, field=[1, 1], r_max=0, gamma=0, n=1, mob=[0, 1, 1])
        self.O1.Defpow = 5e6
        self.O1.side = 'Obstacle'

        self.O2 = direct_fire(x0=Obstacle_pos[1, 0], y0=Obstacle_pos[1, 1], v0=0, type1="O", type2="", firing_rate=0, m=0,
                              shoot_v=0, R_err=1, theta=0, alpha=0, val=0, field=[1, 1], r_max=0, gamma=0, n=1, mob=[0, 1, 1])
        self.O2.Defpow = 5e6
        self.O2.side = 'Obstacle'

        self.O3 = direct_fire(x0=Obstacle_pos[0, 0], y0=Obstacle_pos[0, 1], v0=0, type1="O", type2="", firing_rate=0, m=0,
                              shoot_v=0, R_err=1, theta=0, alpha=0, val=0, field=[1, 1], r_max=0, gamma=0, n=1, mob=[0, 1, 1])
        self.O3.Defpow = 5e6
        self.O3.side = 'Obstacle'

    def init_graph(self):
        '''
        初始化态势图
        '''
        self.attack_graph_host = np.zeros(
            (row_range_graph, column_range_graph))
        self.attack_graph_enemy = np.zeros(
            (row_range_graph, column_range_graph))
        self.defend_graph_host = np.zeros(
            (row_range_graph, column_range_graph))
        self.defend_graph_enemy = np.zeros(
            (row_range_graph, column_range_graph))

    def update_graph(self):
        '''
        根据更新后的agent信息，更新态势图；
        由于graph仅会被Receive线程读写，故不需要加锁
        '''
        for i in range(row_range_graph):
            for j in range(column_range_graph):
                # xx,yy为真实位置
                xx = j*length
                yy = ylim_graph-i*length

                point_atk_host = (
                    direct_E(self.D1, [xx, yy])+indirect_E(self.I1, [xx, yy]))
                self.attack_graph_host[i][j] = point_atk_host

                point_atk_enemy = (
                    direct_E(self.d1, [xx, yy])+indirect_E(self.i1, [xx, yy]))
                self.attack_graph_enemy[i][j] = point_atk_enemy

                point_defend_host = (
                    defend_E(self.D1, [xx, yy])+defend_E(self.C1, [xx, yy])+defend_E(self.I1, [xx, yy])+defend_E(self.O1, [xx, yy])+defend_E(self.O2, [xx, yy])+defend_E(self.O3, [xx, yy]))
                self.defend_graph_host[i][j] = point_defend_host

                point_defend_enemy = (
                    defend_E(self.d1, [xx, yy])+defend_E(self.c1, [xx, yy])+defend_E(self.i1, [xx, yy])+defend_E(self.O1, [xx, yy])+defend_E(self.O2, [xx, yy])+defend_E(self.O3, [xx, yy]))
                self.defend_graph_enemy[i][j] = point_defend_enemy

    def graph_to_map_msg(self, graph, map_msg, graph_name):
        '''
        将态势图转化为待发送的msg，函数内均为局部变量
        '''
        graph = np.array(graph)
        cv2.normalize(graph, graph, 0, 255, cv2.NORM_MINMAX)
        graph_color = cv2.applyColorMap(
            graph.astype(np.uint8), cv2.COLORMAP_HOT)

        # # 画图
        # cv2.namedWindow('Send '+graph_name, cv2.WINDOW_NORMAL |
        #                 cv2.WINDOW_KEEPRATIO)
        # cv2.imshow('Send '+graph_name, graph_color)
        # cv2.waitKey(100)

        map = map_msg.map
        for i in range(row_range_msg):
            row = map.rows.add()
            for j in range(column_range_msg):
                pixel = row.pixels.add()
                if i >= row_offset and j >= column_offset and i < row_range_graph + row_offset and j < column_range_graph + column_offset:
                    pixel.B = graph_color[i-row_offset][j-column_offset][0]
                    pixel.G = graph_color[i-row_offset][j-column_offset][1]
                    pixel.R = graph_color[i-row_offset][j-column_offset][2]
                else:
                    pixel.R = 0
                    pixel.G = 0
                    pixel.B = 0
        return map_msg

    def update_msg(self):
        '''
        更新各msg；
        #! 由于msg会同时被Send线程读取，故调用前务必加锁
        '''
        self.agt_list_msg = Situ_Visual_pb2.AgtList(timestamp=self.time_stamp)
        self.atk_host_msg = Situ_Visual_pb2.SituMap(timestamp=self.time_stamp)
        self.atk_enemy_msg = Situ_Visual_pb2.SituMap(timestamp=self.time_stamp)
        self.def_host_msg = Situ_Visual_pb2.SituMap(timestamp=self.time_stamp)
        self.def_enemy_msg = Situ_Visual_pb2.SituMap(timestamp=self.time_stamp)
        self.pred_short_msg = Situ_Visual_pb2.SituMap(
            timestamp=self.time_stamp)
        self.pred_long_msg = Situ_Visual_pb2.SituMap(timestamp=self.time_stamp)

        self.atk_host_msg = self.graph_to_map_msg(
            self.attack_graph_host, self.atk_host_msg, "atk_host")
        self.atk_enemy_msg = self.graph_to_map_msg(
            self.attack_graph_enemy, self.atk_enemy_msg, "atk_enemy")
        self.def_host_msg = self.graph_to_map_msg(
            self.defend_graph_host, self.def_host_msg, "def_host")
        self.def_enemy_msg = self.graph_to_map_msg(
            self.defend_graph_enemy, self.def_enemy_msg, "def_enemy")
        self.pred_short_msg = self.graph_to_map_msg(
            self.attack_graph_host, self.pred_short_msg, "pred_short")
        self.pred_long_msg = self.graph_to_map_msg(
            self.attack_graph_host, self.pred_long_msg, "pred_long")

        agent_list = [self.D1, self.C1, self.I1, self.d1,
                      self.c1, self.i1, self.O1, self.O2, self.O3]
        for agent in agent_list:
            agt_msg = self.agt_list_msg.agts.add()

            # FIXME: 和华如确认，此处单位为米，还是态势矩阵的栅格
            agt_msg.x = int(agent.x)
            agt_msg.y = int(agent.y)
            # agt_msg.x = int(agent.x/length) + column_offset
            # agt_msg.y = int(agent.y/length) + row_offset

            agt_msg.type = agent.side
            agt_msg.damage = Damage(agent, 1, self.attack_graph_host)
            agt_msg.life = Life(agent, 1, self.defend_graph_enemy)
            agt_msg.p_destory = P_des(agt_msg.damage, agt_msg.life)
            agt_msg.atk_benefit = benefit(agent, agt_msg.damage, agt_msg.life)

    def update(self, agent_code, position, yaw):
        '''
        更新对应智能体的位姿，并更新全局态势图
        '''
        id = self.codeList.index(agent_code)

        # update agents position and yaw
        if id == 0:
            self.D1.x0 = position[0]
            self.D1.y0 = position[1]
            self.D1.theta = yaw  # FIXME:向工程师确认朝向的角度单位
        if id == 1:
            self.C1.x0 = position[0]
            self.C1.y0 = position[1]
            self.C1.theta = yaw  # FIXME:向工程师确认朝向的角度单位
        if id == 2:
            self.I1.x0 = position[0]
            self.I1.y0 = position[1]
            self.I1.theta = yaw  # FIXME:向工程师确认朝向的角度单位

        # update graph
        self.update_graph()

        lock.acquire()

        # update msg
        self.update_msg()
        self.time_stamp = self.time_stamp + 1

        lock.release()
        
        print("\n\nagent code: %s \nx: %d \ny: %d \nyaw: %d" % (
            agent_code, position[0], position[1], yaw))
        
        global SEND_FLAG
        SEND_FLAG = True


class DataTransfServicer(data_transf_pb2_grpc.DataTransfServiceServicer):
    def ZNTStatusReceive(self, request, context):
        if request.zntCode not in agent_list.codeList:  # 若传来的智能体并非D1，C1，I1，会触发Error
            logging.error("zntCode is not in agent list")
            response = data_transf_pb2.BaseRespInfo(
                code='-1',
                msg="zntCode is not in agent list",
                # serverTime= # FIXME: 时间戳应该写啥
            )
        else:  # 更新对应智能体的位姿，更新全局态势图，并回传proto所要求格式回应
            print("Receive agent", request.zntCode)
            agent_list.update(
                request.zntCode, request.zntPosition, request.hxAngle)  # FIXME: 向工程师确认编码格式，朝向的角度单位

            response = data_transf_pb2.BaseRespInfo(
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


def Send_Agts():
    with grpc.insecure_channel(IP_VISUAL+':'+PORT_SITU2VISUAL_AGTS) as channel_agts:
        stub_agts = Situ_Visual_pb2_grpc.Situ_VisualStub(channel_agts)
        lock.acquire()
        print("Sending Agent List", agent_list.time_stamp)
        response = stub_agts.agent_list(agent_list.agt_list_msg)
        lock.release()
        print(response.data)


def Send_Atk_Host():
    with grpc.insecure_channel(IP_VISUAL+':'+PORT_SITU2VISUAL_ATK_HOST) as channel_atk_host:
        stub_atk_host = Situ_Visual_pb2_grpc.Situ_VisualStub(channel_atk_host)
        lock.acquire()
        print("Sending atk_host_msg", agent_list.time_stamp)
        response = stub_atk_host.atk_host(agent_list.atk_host_msg)
        lock.release()
        print(response.data)


def Send_Atk_Enemy():
    with grpc.insecure_channel(IP_VISUAL+':'+PORT_SITU2VISUAL_ATK_ENEMY) as channel_atk_enemy:
        stub_atk_enemy = Situ_Visual_pb2_grpc.Situ_VisualStub(
            channel_atk_enemy)
        lock.acquire()
        print("Sending atk_enemy_msg", agent_list.time_stamp)
        response = stub_atk_enemy.atk_enemy(agent_list.atk_enemy_msg)
        lock.release()
        print(response.data)


def Send_Def_Host():
    with grpc.insecure_channel(IP_VISUAL+':'+PORT_SITU2VISUAL_DEF_HOST) as channel_def_host:
        stub_def_host = Situ_Visual_pb2_grpc.Situ_VisualStub(channel_def_host)
        lock.acquire()
        print("Sending def_host_msg", agent_list.time_stamp)
        response = stub_def_host.def_host(agent_list.def_host_msg)
        lock.release()
        print(response.data)


def Send_Def_Enemy():
    with grpc.insecure_channel(IP_VISUAL+':'+PORT_SITU2VISUAL_DEF_ENEMY) as channel_def_enemy:
        stub_def_enemy = Situ_Visual_pb2_grpc.Situ_VisualStub(
            channel_def_enemy)
        lock.acquire()
        print("Sending def_enemy_msg", agent_list.time_stamp)
        response = stub_def_enemy.def_enemy(agent_list.def_enemy_msg)
        lock.release()
        print(response.data)


def Send_Pred_Short():
    with grpc.insecure_channel(IP_VISUAL+':'+PORT_SITU2VISUAL_PRED_SHORT) as channel_pred_short:
        stub_pred_short = Situ_Visual_pb2_grpc.Situ_VisualStub(
            channel_pred_short)
        lock.acquire()
        print("Sending pred_short_msg", agent_list.time_stamp)
        response = stub_pred_short.pred_short(agent_list.pred_short_msg)
        lock.release()
        print(response.data)


def Send_Pred_Long():
    with grpc.insecure_channel(IP_VISUAL+':'+PORT_SITU2VISUAL_PRED_LONG) as channel_pred_long:
        stub_pred_long = Situ_Visual_pb2_grpc.Situ_VisualStub(
            channel_pred_long)
        lock.acquire()
        print("Sending pred_long_msg", agent_list.time_stamp)
        response = stub_pred_long.pred_long(agent_list.pred_long_msg)
        lock.release()
        print(response.data)


def Send_to_Visual():
    '''
    向可视化模块发送数据，各个Send_函数通过线程定时器实现循环
    '''
    global SEND_FLAG

    if SEND_FLAG is True:
        t_send_agts = Thread(target=Send_Agts)
        t_send_agts.start()
        t_send_atk_host = Thread(target=Send_Atk_Host)
        t_send_atk_host.start()
        t_send_atk_enemy = Thread(target=Send_Atk_Enemy)
        t_send_atk_enemy.start()
        t_send_def_host = Thread(target=Send_Def_Host)
        t_send_def_host.start()
        t_send_def_enemy = Thread(target=Send_Def_Enemy)
        t_send_def_enemy.start()
        t_send_pred_short = Thread(target=Send_Pred_Short)
        t_send_pred_short.start()
        t_send_pred_long = Thread(target=Send_Pred_Long)
        t_send_pred_long.start()

        SEND_FLAG = False


if __name__ == '__main__':
    logging.basicConfig()

    # # 请求智能体数据
    # Ask_Data()

    # 初始化全局变量，智能体队列
    global agent_list
    agent_list = Agent_List()

    # 启动各线程
    thd_rec = Thread(target=Rec_from_Core)
    thd_rec.start()
    thd_send = RepeatingTimer(Timer_Interval, Send_to_Visual)
    thd_send.start()
