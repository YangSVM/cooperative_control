#!/usr/bin/env python3

"""
v 1.0

还需要增加功能：
1. GRPC读取。(建议单独写一个test函数。保留现有的使用ros的仿真结果)

希望增加功能：
1. 编队保持状态的反馈控制：(根据现有位置和理想编队位置的偏差，发布速度指令进行控制)

"""
from formation_core import FormationState
from operator import pos
import matplotlib
from numpy.testing._private.utils import print_assert_equal

from scipy.optimize.optimize import vecnorm
from formation_core import Assign
import rospy
import numpy as np
from trajectory_tracking.msg import Trajectory,RoadPoint
from formation_zoo import *
from math import pi, trunc
from formation_core import local_traj_gen, FormationROS, cartesian2frenet
import cubic_spline_planner
import threading
import time
import math


PI = pi
V = 1.5         # m/s
MIN_R = 0.8/np.tan(20/180*PI)           # 2.2米左右
task = 1
n_car = 3
car_id = [1,2,5]


# 前三个为敌方车辆，后三个为敌方智能体
obs = np.array([[ -7.49299999995856,	16.2839999999851],
    [-10.7550000000047,	16.5529999993742],
    [-11.8640000000014,	13.4799999995157],
    [-7.98099999997066,17.2149999998510],
    [-11.3619999999646,17.3819999992847],
    [-12.5129999999772,14.2319999998435]])

# 演示场景中，我方静止队形时的位置
battle_pos=np.array([
    [-3.25699999998324,11.5859999991953],
    [-5.77099999994971,10.5299999993294],
    [-7.87099999998463,8.97999999951571],
])
# 场地边界
boundary = np.array([[12.06,19.49],
    [4.51,19.77],
    [-22.23,19.89],
    [-22.68,-0.44],
    [-15.10,-18.49],
    [-6.83,-22.89],
    [11.80,-22.88],
    [13.49,-10.47],
    [13.70,14.47],
    [12.20,19.50]])
    
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

        if i_task==1:
            pos_formations = load_scene_summon()
        elif i_task==2:
            pos_formations = load_scene_battle_forward()
        elif i_task==3:
            pos_formations, obs = load_scene_battle_with_obstacle()
        elif i_task==4:
            pursuit()
        else:
            print('Unkown Task type')
            return
        self.obs = obs
        self.pos_formations = pos_formations
        self.global_frenet_csp = 0          # 没有用到的特征。之后可能删除掉。

        # 队形点的frenet坐标。这个时间为20ms，得提前算
        self.pos_formations_frenet = self.calc_pos_frenet(pos_formations)


    def calc_pos_frenet(self, pos_formations):
        # 计算队形点的s-d frenet坐标。统一运算，避免重复计算
        pos_formations_frenet = np.empty(pos_formations.shape)
        for i_car in range(n_car):
            csp = cubic_spline_planner.Spline2D(pos_formations[:, i_car,0], pos_formations[:, i_car,1])
            for i_stage in range(pos_formations.shape[0]):
                sd_i_stage = cartesian2frenet(pos_formations[i_stage, i_car, 0], pos_formations[i_stage, i_car, 1], csp)
                pos_formations_frenet[i_stage, i_car, :] = sd_i_stage
        
        return pos_formations_frenet

    def recalc_pos_frenet(self, states):
        ''' 编队
        '''
        # 将states转成numpy
        n_car = self.pos_formations.shape[1]
        states_np = np.empty([n_car, 2])
        for i_car in range(n_car):
            states_np[i_car, 0] = states[i_car].position.x
            states_np[i_car, 1] = states[i_car].position.y
        pos_formations = self.pos_formations
        pos_formations_frenet = np.empty(pos_formations.shape)

        pos_formations[0, :] = Assign(states_np, pos_formations[0, :])
        # 重新分配
        for i_stage in range(1, pos_formations.shape[0]):
            pos_formations[i_stage, :] = Assign(pos_formations[i_stage-1, :], pos_formations[i_stage, :])

        # 计算队形点的s-d frenet坐标。统一运算，避免重复计算
        for i_car in range(n_car):
            csp = cubic_spline_planner.Spline2D(pos_formations[:, i_car,0], pos_formations[:, i_car,1])
            for i_stage in range(pos_formations.shape[0]):
                sd_i_stage = cartesian2frenet(pos_formations[i_stage, i_car, 0], pos_formations[i_stage, i_car, 1], csp)
                pos_formations_frenet[i_stage, i_car, :] = sd_i_stage
        
        self.pos_formations_frenet = pos_formations_frenet

    def plan(self, states, formation_ros: FormationROS):    
        '''集结！固定路线无障碍物编队

        '''
        # 分成两阶段进行：第一阶段初始化，后续持续进行
        # 设计编队集合点 。一字队形->三角队形，一字队形。

        path_x, path_y, path_v = local_traj_gen(self.pos_formations, self.obs, states, self.global_frenet_csp, formation_ros, self.pos_formations_frenet)

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
    



def load_scene_battle_with_obstacle():
    ''' 打击！通过锥桶逐个打击.障碍物。锥桶位置都修改。暂定只保留
    '''
    pos_formations = []
    start_pos = np.array([[-3,11],
        [-5,10],
        [-7,9],
    ])
    pos_formations.append(start_pos)

    delta_line = 4
    center0 = start_pos[1, :]
    vec_norm = np.array([-1, 2])/np.sqrt(5)
    vec_line = np.array([2, 1])/np.sqrt(5)
    degree_norm = math.atan2(vec_norm[1], vec_norm[0])*180/pi
    degree_line = math.atan2(vec_line[1], vec_line[0])*180/pi
    center1 = center0 + vec_norm*delta_line + vec_line * 2     # 
    pos_line = formation_line(center1, degree_norm, n_car, 2)
    pos_formations.append(pos_line)
    
    center2 = center0 + vec_norm*delta_line*2
    pos_v_line  = formation_line(center2, degree_line, n_car, 2)
    pos_formations.append(pos_v_line)
    pos_formations = np.array(pos_formations)

    # 目标分配
    for i_stage in range(1, pos_formations.shape[0]):
        pos_formations[i_stage, :] = Assign(pos_formations[i_stage-1, :], pos_formations[i_stage, :])
    
    center_obs = center0 + vec_norm*delta_line
    obs = formation_line(center_obs, degree_line, 3, 4)
    return pos_formations, obs


def load_scene_summon():
    '''集结场景
    '''
    # 设计轨迹
    t1 = time.time()
    r = 12
    r_u_turn = 3
    center_line = np.array([
        [0,r+5],
        [0, r+5 -7],
        [0, -r + r_u_turn],
        [r_u_turn, -r],
        [2*r_u_turn, -r + r_u_turn],
        [2*r_u_turn, -r + r_u_turn + 2],
    ])
    
    pos_formations = []
    pos_line =formation_line(center_line[0, :], 0, n_car, 2)
    pos_formations.append(pos_line)

    # 三角队形匀速开始
    pos_tri = formation_triangle(center_line[1, :],- 90,n_car,  2)
    pos_formations.append(pos_tri)

    # 三角队形匀速控制点
    n_contrl = 3
    d_dy = (center_line[2, :]- center_line[1, :])/(n_contrl + 1)
    for i_contrl in range(n_contrl):
        pos_constant_ = formation_triangle(center_line[1, :]+d_dy*(i_contrl+1) ,- 90,n_car,  2)
        pos_formations.append(pos_constant_)

    # 匀速运动结束位置
    pos_tri_end = formation_triangle(center_line[2, :], -90,n_car,  2)
    pos_formations.append(pos_tri_end)
    # 转弯中间位置
    pos_turn = formation_triangle(center_line[3, :], 0,n_car,  2)
    pos_formations.append(pos_turn)
    # 结束点位置
    pos_end = formation_line(center_line[4, :], 0, n_car, 2)
    pos_formations.append(pos_end)

    pos_end_ = formation_line(center_line[5, :], 0, n_car, 2)
    pos_formations.append(pos_end_)
    #     
    # pos_formations: (n_formation, n_car, 2)。表示共有几个队形阶段。
    pos_formations = np.array(pos_formations)

    # 目标分配
    for i_stage in range(1, pos_formations.shape[0]):
        pos_formations[i_stage, :] = Assign(pos_formations[i_stage-1, :], pos_formations[i_stage, :])

    t2 = time.time()
    print('previous prepare time: ', t2-t1)
    return pos_formations


def load_scene_battle_forward():
    '''编队前去指定地点
    '''
        # 设计轨迹
    r = 12
    r_u_turn = 3
    center_line = np.array([
        [2*r_u_turn, -r + r_u_turn + 2],
    ])
    start_pos = np.array([[-3,11],
        [-5,10],
        [-7,9],
    ])
    end_pos = start_pos

    pos_formations = []
    d_car = 2
    # 加一个结束点，使得车头朝北
    pos_start_= formation_line(center_line[0, :], 0, n_car, d_car)
    pos_formations.append(pos_start_)

    # 需要再中间再加几个编队。

    pos_formations.append(end_pos)

    # pos_formations: (n_formation, n_car, 2)。表示共有几个队形阶段。
    pos_formations = np.array(pos_formations)

    # 目标分配
    for i_stage in range(1, pos_formations.shape[0]):
        pos_formations[i_stage, :] = Assign(pos_formations[i_stage-1, :], pos_formations[i_stage, :])

    return pos_formations
    

def pursuit():
    '''围捕！
    '''
    pass

def main4ros():

    formation_ros = FormationROS(3)
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

        # task transition
        isTaskFinished = True
        for i_car in range(n_car):
            current_state =formation_ros.vehicle_states[i_car]
            if  current_state != FormationState.goal and current_state != FormationState.waiting:
                isTaskFinished = False
                break
        if isTaskFinished:
            if task == 3:           # TODO debug mode
                print('Task all finished')
                
                formation_ros.rate.sleep()
                continue
            else:
                task += 1
                formation_ros.vehicle_states = [FormationState.running for i in range(n_car)]
                formation_ros.vehicle_formation_stage = [0 for i in range(n_car)]
                task_planners[task].recalc_pos_frenet(states)


        print('-'*30, task, '-'*30)
        if task ==0:
            traj1, traj2, traj3 = task_planners[task]
        elif task==1:

            path_x, path_y, path_v = task_planners[task].plan(states, formation_ros)
        elif task==2:
            path_x, path_y, path_v = task_planners[task].plan(states, formation_ros)
        elif task==3:
            path_x, path_y, path_v = task_planners[task].plan(states, formation_ros)
        elif task==4:
            pursuit()
        if task !=0 :
            #  非循迹
            formation_ros.publish(path_x, path_y, path_v )
        else:
            formation_ros.pub_traj([traj1, traj2, traj3] )

        t2=time.time()
        print('总时间：', t2-t1)
        formation_ros.rate.sleep()

    


if __name__ == '__main__':
    main4ros()
    # pos, obs = load_scene_battle_with_obstacle()
    # pos = load_scene_summon()
    # plot4test(pos,obs)
    