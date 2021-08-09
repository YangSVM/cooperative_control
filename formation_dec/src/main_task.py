#!/usr/bin/env python3

"""
v 0.1
"""
from formation_core import Assign
import rospy
import numpy as np
from trajectory_tracking.msg import Trajectory,RoadPoint
from formation_zoo import *
from math import pi, trunc
from formation_core import local_traj_gen, FormationROS
import cubic_spline_planner
import threading
import time


PI = pi
V = 1.5         # m/s
MIN_R = 0.8/np.tan(20/180*PI)           # 2.2米左右
task = 1
n_car = 3
car_id = [1,2,5]


obs = np.array([[ -7.49299999995856,	16.2839999999851],
    [-10.7550000000047,	16.5529999993742],
    [-11.8640000000014,	13.4799999995157]])

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


def summon(states, formation_ros: FormationROS):
    '''集结！编队

    '''
    # 设计编队集合点 。一字队形->三角队形，一字队形。
    r = 12
    r_u_turn = 3
    center_line = np.array([
        [0,r+5],
        [0, r+5 -7],
        [0, -r + r_u_turn],
        [r_u_turn, -r],
        [2*r_u_turn, -r + r_u_turn],
    ])
    
    global_frenet_csp = cubic_spline_planner.Spline2D(center_line[:, 0], center_line[:, 1])
    
    pos_line =formation_line(center_line[0, :], 0, n_car, 2)
    
    pos_tri = formation_triangle(center_line[1, :],- 90,n_car,  2)
    
    # 匀速运动结束位置
    pos_tri_end = formation_triangle(center_line[2, :], -90,n_car,  2)

    # 转弯中间位置
    pos_turn = formation_triangle(center_line[3, :], 0,n_car,  2)

    # 结束点位置
    pos_end = formation_line(center_line[4, :], 0, n_car, 2)

    # pos_formations: (n_formation, n_car, 2)。表示共有几个队形阶段。
    pos_formations = np.array([pos_line, pos_tri, pos_tri_end, pos_turn, pos_end])
    # 目标分配
    for i_stage in range(1, pos_formations.shape[0]):
        pos_formations[i_stage, :] = Assign(pos_formations[i_stage-1, :], pos_formations[i_stage, :])


    path_x, path_y, path_v = local_traj_gen(pos_formations, obs, states, global_frenet_csp, formation_ros)

    return path_x, path_y, path_v


def battle():
    '''打击！
    '''


def pursuit():
    '''围捕！
    '''


def main():

    formation_ros = FormationROS(3)

    while not rospy.is_shutdown():
        t1 = time.time()
        states = formation_ros.get_pose_states()

        global task
        if task ==0:
            traj1, traj2, traj3 = search()
        elif task==1:
            if states == None:
                continue
            path_x, path_y, path_v = summon(states, formation_ros)
        elif task==2:
            battle()
        elif task==3():
            pursuit()
        formation_ros.publish(path_x, path_y, path_v )
        # formation_ros.pub_traj(traj2)
        t2=time.time()
        print('time', t2-t1)
        formation_ros.rate.sleep()

    


if __name__ == '__main__':
    main()