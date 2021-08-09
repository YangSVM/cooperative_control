#!/usr/bin/env python3

"""
v 0.1
"""
# from formation_core import Assign
import rospy
import numpy as np
from trajectory_tracking.msg import Trajectory,RoadPoint
from formation_zoo import *
from math import pi



PI = pi
V = 1.5         # m/s
MIN_R = 0.8/np.tan(20/180*PI)           # 2.2米左右
task = 0
n_car = 3
car_id = [1,2,5]

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
    y_line = np.arange(-r, 0, 0.05)
    arc2 = generate_arc([MIN_R, 0], 180, 0, MIN_R, -1)
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




def battle():
    '''打击！
    '''


def pursuit():
    '''围捕！
    '''


def main():

    rospy.init_node('test4sth')
    pub = rospy.Publisher('car1'+'/local_trajectory', Trajectory, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():


        traj1, traj2, traj3 = search()

        pub.publish(traj2)
        rate.sleep()


    


if __name__ == '__main__':
    main()