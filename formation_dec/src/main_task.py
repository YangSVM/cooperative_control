#!/usr/bin/env python3

"""
v 1.0

还需要增加功能：
1. 增加grpc ros接口。任务逻辑接口
    - 收到任务指令才开始（仿真这个）
    - 按照给定的时间进行任务(如果任务计划表中的开始时间没到，不开始)
    - 打击实现任务改变
    - 定时返回预计完成时间
2. 疑问：duration是否是end time的意思(不是。有start time)
2. sample strategy

希望增加功能：
1. 编队保持状态的反馈控制：(根据现有位置和理想编队位置的偏差，发布速度指令进行控制)

"""
from os import isatty
from typing import List
from formation_core import FormationState

from formation_core import Assign
import rospy
import numpy as np
from trajectory_tracking.msg import Trajectory,RoadPoint
from formation_dec.formation_zoo import *
from math import pi
from formation_core import local_traj_gen, FormationROS, cartesian2frenet, sequential_passing
import cubic_spline_planner
import time
import math
from communication.msg import ExecInfoRequest, TasksRequest, AttackRequest, TaskEI
from formation_dec.config_formation import *


class FormationWithTask(FormationROS):
    def __init__(self, n_car, car_ids):
        super(FormationWithTask, self).__init__(n_car, car_ids)
        rospy.Subscriber('TasksRequest', TasksRequest, self.task_request_callback)
        rospy.Subscriber('AttackRequest', AttackRequest, self.attack_request_callback)
        # time step for start
        self.init_timestamp = None
        self.init_timestamp_flag = False
        self.tasks_info = None
        self.attack_info_string = None
        # list of dict(stages). dict: key enemy, value: ally.
        self.attack_stages_info = None
        self.pub_tasks_ei = rospy.Publisher('ExecInfoRequest', ExecInfoRequest, queue_size=1)
        self.attack_direction = None

    def task_request_callback(self, msg):
        if not self.init_timestamp_flag:
            self.init_timestamp_flag = True
            self.init_timestamp = msg.tasks[0].init_timestamp

        self.tasks_info = msg
        
   
    def attack_request_callback(self, msg):
        self.attack_info_string = msg
        self.parse_attack_info()

    def parse_attack_info(self):
        # translate the attack string into list of dict. (one ally can attack one ally only)
        self.attack_stages_info = []
        for string in self.attack_info_string:
            attack_pairs = string.split('*')
            stage = {}
            for attack_pair in attack_pairs:
                enemy_id = int(attack_pair[2:])
                ally_id = int(attack_pair[:2])
                if enemy_id in stage.keys():
                    stage[enemy_id].append(ally_id)
                else:
                    stage[enemy_id] = [ally_id]
            self.attack_stages_info.append(stage)
        
        # 根据数组初步判断。看第一个阶段打哪辆车.  TODO 确认敌方第一辆车是4号车。
        
        if 4 in self.attack_stages_info[0].keys():
            self.attack_direction = -1          # 4打左边的车
        elif 5 in self.attack_stages_info[0].keys():
            self.attack_direction = 1           # 5打右边的车
        else:
            rospy.logerr('cannot define attack direction')

    def calc_task_duration(self, task):

        if self.tasks_info is None:
            return
        
        # 假设order按照顺序发送。忽略排序的过程
        exec_info = ExecInfoRequest()
        task_time = [100,200,300,400,500]
        time_now = time.time() - self.init_timestamp
        for task_info in self.tasks_info.tasks:
            task_ei = TaskEI()
            task_ei.order = task_info.order
            task_type = task_info.type
            if task_type >task:
                task_ei.status = -1
            elif task_type == task:
                task_ei.status = 1
            elif task_type < task:
                task_ei.status = 0
            
            task_ei.duration_upd = task_time[task] - time_now 
            exec_info.tasks_ei.append(task_ei)


        self.pub_tasks_ei(exec_info)
        pass

    def judge_next_task(self, pretask):
        ''' 根据TasksRequest开始信息进行。
        '''
        assert self.init_timestamp_flag, 'task not init. donot have init_timestamp'
        time_now = time.time() - self.init_timestamp
        res = True
        if time_now < self.tasks_info.tasks[pretask+1].start_time:
            res=False
        return res


def pack_trajectory(roadpoints, pub_v=True):
    # packing list/numpy to ros Trajectory
    n_points = len(roadpoints)
    traj = Trajectory()
    for i in range(n_points):
        rp = RoadPoint()
        rp.x =roadpoints[i][0]
        rp.y = roadpoints[i][1]
        # 默认包含速度信息。否则直接为0
        if pub_v:
            rp.v = roadpoints[i][2]

        traj.roadpoints.append(rp)
    return traj

def generate_arc(center, theta_s, theta_e, r, isCounterClock=1):
    ''' 生成一条圆弧
    Params:
        theta_s: start. degree
        theta_e: end. degree
        isCounterClock: 逆时针为正。否则为负。
    Return:
        np.array. [n, 3]: x,y,v
    '''
    theta = np.arange(theta_s/180*PI, theta_e/180*PI, isCounterClock*0.05/r)       # 保证大概1cm一个点
    roadpoints = np.empty([len(theta), 3])
    roadpoints[:,0] = r*np.cos(theta) + center[0]
    roadpoints[:,1] = r*np.sin(theta) + center[1]
    roadpoints[:,2] = V

    return roadpoints


def search():
    '''侦查！！！
    目前是手动设计圆弧。可以用样条曲线搞一下。
    '''

    r = 14
    # car 1 一个半圆搜索
    arc1 = generate_arc([0,0], -90,-90+360+180, r)                # 11310个点.太多了
    traj1 = pack_trajectory(arc1)


    # car2 直线掉头
    y_line = np.arange(-r, r, 0.05)
    arc2 = generate_arc([MIN_R, r], 180, 0, MIN_R, -1)
    n_rp2 = len(y_line) + arc2.shape[0]

    roadpoints2 = np.empty([n_rp2, 3])
    roadpoints2[:len(y_line), 0] = 0
    roadpoints2[:len(y_line), 1] = y_line
    
    roadpoints2[len(y_line):, :] = arc2
    roadpoints2[:, 2] = V
    traj2 = pack_trajectory(roadpoints2)

    # car3 zuo半圆搜索
    arc3 = generate_arc([0,0], -90, -90-180, r, -1)
    traj3 = pack_trajectory(arc3)
    return traj1, traj2, traj3

 
class TaskBase():
    def __init__(self, i_task, states=None, insert_pos=0):
        global obs
        self.task = i_task
        self.critical_state_selector = None
        if i_task==1:           # 集结
            pos_formations, self.theta_degs, self.critical_state_selector = load_scene_summon()
        elif i_task==2:         # 到锥桶前
            pos_formations, self.theta_degs = load_scene_prebattle()
        elif i_task==3:         # 通过障碍
            # pos_formations, obs = load_scene_battle()
            # pos_formations,  self.theta_degs = load_avoid_summon()
            pos_formations,  self.theta_degs, self.critical_state_selector = load_sequence_passing()

            # plt.plot(pos_formations[..., 0], pos_formations[...,1], 'r*')
            # plt.axis('square')
            # plt.show()
        else:
            print('Unkown Task type')
            return
        self.obs = obs

        self.pos_formations = pos_formations

        # 队形点的frenet坐标。这个时间为20ms，得提前算
        # self.pos_formations_frenet = self.calc_pos_frenet(pos_formations, self.global_frenet_csps)
        # 有的场景需要重新进行匈牙利指派
        self.is_repost_process = False


    def calc_pos_frenet(self, pos_formations, global_frenet_csps):
        # 计算队形点的s-d frenet坐标。统一运算，避免重复计算：大量减少时间复杂度
        pos_formations_frenet = np.empty(pos_formations.shape)
        for i_car in range(n_car):
            csp = global_frenet_csps[i_car]
            for i_stage in range(pos_formations.shape[0]):
                sd_i_stage = cartesian2frenet(pos_formations[i_stage, i_car, 0], pos_formations[i_stage, i_car, 1], csp)
                pos_formations_frenet[i_stage, i_car, :] = sd_i_stage
        
        return pos_formations_frenet

    def repost_process(self, states):
        ''' 重新进行后处理。每个阶段需要重新拼接
        '''
        # 将states转成numpy. 并进行拼接
        n_car = self.pos_formations.shape[1]
        states_np = np.empty([1, n_car, 2])
        for i_car in range(n_car):
            states_np[0, i_car, 0] = states[i_car].position.x
            states_np[0, i_car, 1] = states[i_car].position.y

        pos_formations_extend = np.concatenate((states_np, self.pos_formations), axis=0)

        self.pos_formations, self.global_frenet_csps, self.pass_orders  = scene_post_process(pos_formations_extend, 
                                                            self.theta_degs,global_frenet_csps=[], critical_state_selector=self.critical_state_selector)


        self.pos_formations_frenet = self.calc_pos_frenet(self.pos_formations, self.global_frenet_csps)


    def plan(self, states, formation_ros: FormationROS):    
        '''决策规划。根据当前state生成一条进行符合轨迹的

        '''
        sample_basis = None
        if not self.is_repost_process:
            self.is_repost_process = True
            self.repost_process(states)
        
        if self.task == 3:
            path_x, path_y, path_v = sequential_passing(self.pos_formations, states, self.global_frenet_csps, self.pos_formations_frenet,  formation_ros)
        else:
            path_x, path_y, path_v = local_traj_gen(self.pos_formations, self.pass_orders, self.obs, states, self.global_frenet_csps, formation_ros, self.pos_formations_frenet, sample_basis)

        return path_x, path_y, path_v


def plot4test(pos_formations, obs):

    n_stage = pos_formations.shape[0]
    plt.figure()
    for i_stage in range(n_stage):
        plt.plot(pos_formations[i_stage, :, 0], pos_formations[i_stage, :, 1], 'r*')
    plt.plot(obs[:, 0], obs[:,1], 'bo')
    plt.plot(boundary[:,0], boundary[:,1], 'r-')    
    plt.axis('square')
    plt.show()
    

def scene_post_process(pos_formations:np.ndarray, theta_degs, critical_state_selector:List = None, global_frenet_csps:List = [], is_include_ego=True):
    '''编辑场景后需要的后处理。包括匈牙利指派，生成全局路径，提取队形关键阶段点（提前到达的车辆需要等后续车辆）
        得出通行次序。
    params:
        pos_formations: 自车当前状态+编辑的队形点
        critical_state_selector: list. 关键点是当中的哪几个. 如果是默认值，则全选
        global_frenet_csps： 全局轨迹。如果是默认值[]，则根据 pos_formation生成全局轨迹；如果非空，则是repost_process.
            此时第一个阶段点是车辆目前状态，必须忽略(目前状态和第一个阶段点太近，样条曲线稳定性太差)
    '''

    # 计算通行次序。从1开始。pass_orders的长度比控制点数目少
    pass_orders = []
    for i_stage in range(1, pos_formations.shape[0]-1):
        pass_order = calc_pass_order(pos_formations[i_stage+1, ...], theta_degs[i_stage])
        pass_orders.append(pass_order)
    pass_orders = np.array(pass_orders)

    # 目标分配
    for i_stage in range(1, pos_formations.shape[0]):
        pos_formations[i_stage, :], ori_id, sorted_id= Assign(pos_formations[i_stage-1, :], pos_formations[i_stage, :] )
        if i_stage >  pos_formations.shape[0]-1 or i_stage<2:
            continue
        pass_orders[i_stage-2, ori_id] = pass_orders[i_stage-2, sorted_id]

    # 生成全局路径，并设置关键编队点
    n_car = pos_formations.shape[1]

    if is_include_ego:
        # 如果没有输入 global_frenet_csps，则直接根据 pos_formations生成
        start_id = 1
    else:
        # 如果输入非空，需要重新进行排列。忽略第一个个队形点（自车状态点）
        start_id = 0

    global_frenet_csps = []
    for i_car in range(n_car):
        csp = cubic_spline_planner.Spline2D(pos_formations[start_id:, i_car,0], pos_formations[start_id:, i_car,1])
        global_frenet_csps.append(csp)
    
    #
    # 关键点采集。先去除自车状态，再选择原来的点
    pos_formations = pos_formations[1:, ...]
    if critical_state_selector is None:
        critical_state_selector = [i for i in range(pos_formations.shape[0])]
    pos_formations = pos_formations[critical_state_selector,...]
        
    return pos_formations, global_frenet_csps, pass_orders


def load_scene_summon():
    '''集结场景
    '''
    # 设计轨迹
    t1 = time.time()

    pos_formations = []
    pass_orders = []

    pos_line =formation_line(center_line[0, :], 0, n_car, d_car)
    pos_formations.append(pos_line)

    # 三角队形匀速开始
    # 三角队形输入
    if n_car%3 == 0 :
        n_edge_car = n_car//3
    else:
        n_edge_car = n_car//3 + 1
    # 外接圆半径计算。原则：每条边上车辆距离应该是定值
    r_tri = (d_car * n_edge_car)/math.sqrt(3) * math.sqrt(3)
    pos_tri = formation_triangle(center_line[1, :],- 90,n_car,  r_tri)
    pos_formations.append(pos_tri)

    # 三角队形匀速控制点
    n_contrl = 3
    d_dy = (center_line[2, :]- center_line[1, :])/(n_contrl + 1)
    for i_contrl in range(n_contrl):
        pos_constant_ = formation_triangle(center_line[1, :]+d_dy*(i_contrl+1) ,- 90,n_car,  r_tri)
        pos_formations.append(pos_constant_)

    # 匀速运动结束位置
    pos_tri_end = formation_triangle(center_line[2, :], -90,n_car,  r_tri)
    pos_formations.append(pos_tri_end)
    # 转弯中间位置
    # pos_turn = formation_triangle(center_line[3, :], 0,n_car,  r_tri)
    # pos_formations.append(pos_turn)
    # 结束点位置
    pos_end = formation_line(center_line[4, :], 0, n_car, d_car)
    pos_formations.append(pos_end)

    pos_end_ = formation_line(center_line[5, :], 0, n_car, d_car)
    pos_formations.append(pos_end_)
   
    # pos_formations: (n_formation, n_car, 2)。表示共有几个队形阶段。
    pos_formations = np.array(pos_formations)
    theta_degs = [-90 for i in range(pos_formations.shape[0] - 3)]
    theta_degs = theta_degs + [0, 90, 90]
    

    critical_state_selector = [0,1,-1]
    # pos_state_formations, global_frenet_csps, pass_orders = scene_post_process(pos_formations, theta_degs,global_frenet_csps=[])

    t2 = time.time()
    print('previous prepare time: ', t2-t1)
    return pos_formations, theta_degs, critical_state_selector


def load_scene_prebattle():
    '''编队前去指定地点
    '''
    # 集结队形设计
    global center_line

    
    global battle_pos, battle_theta
    end_pos_center = np.copy(battle_pos[1, :])
    end_pos_theta = battle_theta

    # end_pos = np.copy(battle_pos)

    pos_formations = []

    # 上一阶段的结束队形
    pos_start_center = center_line[-1, :]
    pos_start_= formation_line(pos_start_center, 0, n_car, d_car)
    pos_formations.append(pos_start_)

    # 先打击一侧，再打击另一侧
    isAttackLeft = 1
    d_theta = 20
    d_pos_x = d_car/2
    if isAttackLeft==1:
        print('attack left')
        d_pos_x = -d_pos_x
    else:
        print('attack right')
        d_theta  = - d_theta
    
    # 中间打一边，最后打另一边
    attack_pos_mid =( pos_start_center+ end_pos_center) /2 + [d_pos_x, 0]

    pos_attack = formation_line(attack_pos_mid, end_pos_theta+d_theta, n_car, d_car)
    pos_formations.append(pos_attack)

    # 终点打击另一辆车
    end_pos = formation_line(end_pos_center+ [-d_pos_x, 0], end_pos_theta-d_theta, n_car, d_car)
    pos_formations.append(end_pos)

    # pos_formations: (n_formation, n_car, 2)。表示共有几个队形阶段。
    pos_formations = np.array(pos_formations)
    theta_degs = [battle_theta for i in range(pos_formations.shape[0])]

    # pos_state_formations, global_frenet_csps, pass_orders = scene_post_process(pos_formations, global_frenet_csps=[], theta_degs=theta_degs)

    return pos_formations, theta_degs
    

def load_avoid_summon():
    '''避障且回到集结点
    '''
    global battle_pos, battle_theta, battle_theta_norm, obs
    pos_formations = []

    # 上一阶段的结束队形
    pos_start_center = np.copy(battle_pos[1, :])
    isAttackLeft = 1
    d_theta = 20
    d_pos_x = d_car/2
    if isAttackLeft==1:
        print('attack left')
        d_pos_x = -d_pos_x
    else:
        print('attack right')
        d_theta  = - d_theta
    pos_start = formation_line(pos_start_center+ [-d_pos_x, 0], battle_theta-d_theta, n_car, d_car)
    pos_formations.append(pos_start)

    # 围绕障碍物的队形
    # 分成两列通过。每列中心是障碍物中心
    pos_through_mid = (obs[:2, :] + obs[1:3, :])/2
    pos_through  = formation_double_line(pos_through_mid, d_car, n_car)
    pos_formations.append(pos_through)

    # 加一个通过障碍物的队形: 通过延伸
    delta_pos = pos_through[1, :] - pos_through[0, :]
    theta = math.atan2(delta_pos[1], delta_pos[0]) 
    theta_norm = theta - np.pi/2
    d_norm = np.array([math.cos(theta_norm), math.sin(theta_norm)])
    d_theta = np.array([math.cos(theta), math.sin(theta)])

    pos_through_next = pos_through + d_norm * 2 + d_theta *(- 0.1)
    # pos_formations.append(pos_through_next)

    # 加一个通过障碍物的队形: 通过增加直线
    
    pos_through_line = formation_line(obs[1, :]+d_norm*3.5, 0, n_car, d_car)
    pos_formations.append(pos_through_line)

    # 返回集结点的队形
    pos_line =formation_line(center_line[0, :], 0, n_car, 2)
    pos_formations.append(pos_line)

    pos_formations = np.array(pos_formations)
    theta_degs = [battle_theta-90 for i in range(pos_formations.shape[0]-2)]
    theta_degs.append(0)
    theta_degs.append(-90)

    # pos_state_formations, global_frenet_csps, pass_orders = scene_post_process(pos_formations, global_frenet_csps=[],theta_degs=theta_degs)

    return pos_formations, theta_degs


def load_sequence_passing():
    '''用直线队列方法通过障碍物
    '''
    global battle_pos, battle_theta, battle_theta_norm, obs
    pos_formations = []

    # 上一阶段的结束队形
    pos_start_center = np.copy(battle_pos[1, :])
    isAttackLeft = 1
    d_theta = 20
    d_pos_x = d_car/2
    if isAttackLeft==1:
        print('attack left')
        d_pos_x = -d_pos_x
    else:
        print('attack right')
        d_theta  = - d_theta
    pos_start = formation_line(pos_start_center+ [-d_pos_x, 0], battle_theta-d_theta, n_car, d_car)
    pos_formations.append(pos_start)

    # 围绕障碍物的队形
    # 单列通过。每列中心是障碍物中心
    theta_norm = (battle_theta_norm)/180*pi
    d_norm =  np.array([math.cos(theta_norm), math.sin(theta_norm)])
    pos_through_mid = (obs[0, :] + obs[1, :])/2 - d_norm *2.5
    pos_through  = formation_point(pos_through_mid, n_car)
    
    pos_formations.append(pos_through)

    # 北尽头拐弯处

    point_north_turn = pos_through_mid+d_norm*3
    pos_north_turn = formation_point(point_north_turn, n_car)
    pos_formations.append(pos_north_turn)

    # 返回集结点的队形
    end_pos = center_line[0, :].copy()
    # control point. 控制点.帮助拐弯
    end_pos_cp = end_pos.copy()
    end_pos_cp[1] += 0.5
    end_pos_cp[0] -= 1
    pos_line_cp =formation_line(end_pos_cp, 0, n_car, d_car)
    pos_formations.append(pos_line_cp)

    pos_line =formation_line(end_pos, 0, n_car, d_car)
    pos_formations.append(pos_line)
    pos_formations = np.array(pos_formations)

    theta_degs = np.array([battle_theta_norm,battle_theta_norm, battle_theta_norm, -90, -90])

    critical_state_selector = [0,1,2,3]
    return pos_formations, theta_degs, critical_state_selector


def load_scene_battle(direction=1):
    ''' [暂时弃用].打击！通过锥桶逐个打击.障碍物。锥桶位置都修改。暂定只保留
    '''
    pos_formations = []
    global battle_pos
    start_pos = np.copy(battle_pos)

    pos_formations.append(start_pos)

    delta_line = 4
    start_pos_center = start_pos[1, :]
    vec_norm = np.array([-1, 2])/np.sqrt(5)
    vec_line = np.array([2, 1])/np.sqrt(5)
    degree_norm = math.atan2(vec_norm[1], vec_norm[0])*180/pi
    degree_line = math.atan2(vec_line[1], vec_line[0])*180/pi
    center1 = start_pos_center + vec_norm*delta_line + vec_line * 2 *direction    # 
    pos_line = formation_line(center1, degree_norm, n_car, d_car)
    pos_formations.append(pos_line)
    
    center2 = start_pos_center + vec_norm*delta_line*2
    pos_v_line  = formation_line(center2, degree_line, n_car, d_car)
    pos_formations.append(pos_v_line)
    pos_formations = np.array(pos_formations)

    # 目标分配
    for i_stage in range(1, pos_formations.shape[0]):
        pos_formations[i_stage, :] = Assign(pos_formations[i_stage-1, :], pos_formations[i_stage, :])
    
    center_obs = start_pos_center + vec_norm*2
    obs = formation_line(center_obs, degree_line, 3, 4)
    return pos_formations, obs



def main4ros():

    formation_ros = FormationWithTask(n_car, car_ids)
    global task

    task_planners = [TaskBase(i+1) for i in range(3)]
    task_planners.insert(0, [search()])

    while not rospy.is_shutdown():
        t1 = time.time()
        states = formation_ros.get_pose_states()

        if states == None:
            # wa
            formation_ros.rate.sleep()
            continue
        
        # if not formation_ros.init_timestamp_flag or t1<formation_ros.init_timestamp:
        #     rospy.logwarn('waiting for Task initial')
        #     formation_ros.rate.sleep()
        #     continue

        # task transition
        isTaskFinished = True       # 当前子任务是否完成
        for i_car in range(n_car):
            current_state =formation_ros.vehicle_states[i_car]
            if  current_state != FormationState.goal and current_state != FormationState.waiting:
                isTaskFinished = False
                break
        if isTaskFinished:
            if task == 3:           # TODO debug mode. 仅仅运行到前进到障碍物的地方
                print('Task all finished')
                
                formation_ros.rate.sleep()
                continue
            else:
                task += 1
                formation_ros.vehicle_states = [FormationState.running for i in range(n_car)]
                formation_ros.vehicle_formation_stage = [0 for i in range(n_car)]

        task_name = ['侦查', '集结', '打击','最终集结']
        print('-'*30,'Task :', task_name[task], '-'*30)
        if task ==0:
            traj1, traj2, traj3 = task_planners[task]
        elif task in [1,2,3]:
            path_x, path_y, path_v = task_planners[task].plan(states, formation_ros)


        if task !=0 :
            #  非循迹
            formation_ros.publish(path_x, path_y, path_v )
        else:
            formation_ros.pub_traj([traj1, traj2, traj3] )

        print('vehicle states:  ', formation_ros.vehicle_states)
        t2=time.time()
        print('总时间：', t2-t1)
        
        formation_ros.calc_task_duration(task)
        formation_ros.rate.sleep()

    


if __name__ == '__main__':
    main4ros()
    # pos, obs = load_scene_battle()
    # plot4test(pos,obs)
    