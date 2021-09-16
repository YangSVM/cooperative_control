#!/usr/bin/env python3
'''
仿真的参考系用的右手系
'''
from operator import sub
import matplotlib.pyplot as plt
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from trajectory_tracking.msg import Trajectory
from math import pi
from geometry_msgs.msg import Pose
from utils.draw_lqr import draw_car
from formation_dec.config_formation import *



# 全局变量。
preview_point = Point(0,0,0)
local_trajectory = Trajectory()

is_preview_point_ready_list = [False for i in range(n_car)]
preview_point_list = [Point(0,0,0) for i in range(n_car)]
local_trajectory_list = [Trajectory() for i in range(n_car)]
is_local_trajectory_ready_list = [False for i in range(n_car)]

global_trajectory_list = [Trajectory() for i in range(n_car)]
is_global_trajectory_ready_list = [False for i in range(n_car)]

pose_states =  [Pose() for i in range(n_car)]
is_gps_ready_list = [False for i in range(n_car)]

formation_points_traj = Trajectory()
is_wp_ready = False

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
    
def trajectory2np(trajectory):
    n_points = len(trajectory.roadpoints)

    path = np.zeros([n_points, 2])
    for i in range(n_points):
        path[i,0] = trajectory.roadpoints[i].x
        path[i,1] = trajectory.roadpoints[i].y
        if trajectory.roadpoints[i].v ==0:
            safe_i_last = max([0, i-1])
            path[i, :] = path[safe_i_last, :]

    return path

def sub_gps_states(msg, i):
    global is_gps_ready_list, pose_states
    is_gps_ready_list[i] = True
    pose_states[i].position.x = msg.pose.pose.position.x
    pose_states[i].position.y = msg.pose.pose.position.y
    
    pose_states[i].orientation.z = msg.twist.twist.angular.z


def get_wp(msg):
    global is_wp_ready, formation_points_traj
    is_wp_ready = True
    formation_points_traj = msg


def transform(x, y, theta=None):
    ''' 坐标系转换。从GPS左手系L1转到大地正常的右手系L2，或者右手系L2转到左手系L1。
    注意 L2->L1和L1->L2的变换恰好都是相同的
    '''
    return y, x, pi/2-theta


def getPrewierPoint(msg, id):
    '''preview_point_list 已经在右手系了
    '''
    global preview_point_list, is_preview_point_ready_list
    is_preview_point_ready_list[id] = True
    preview_point_list[id].x = msg.x
    preview_point_list[id].y = msg.y
    preview_point_list[id].z = msg.z


def get_local_trajectory(msg, id):
    global local_trajectory_list, is_local_trajectory_ready_list
    local_trajectory_list[id] =  msg
    is_local_trajectory_ready_list[id] = True

def get_global_trajectory(msg, id):
    global global_trajectory_list, is_global_trajectory_ready_list
    global_trajectory_list[id] =  msg
    is_global_trajectory_ready_list[id] = True

def get_path_xy(trajectory, is_need_cut=False):
    n_points = len(trajectory.roadpoints)

    path = np.zeros([n_points, 2])
    for i in range(n_points):
        path[i,0] = trajectory.roadpoints[i].x
        path[i,1] = trajectory.roadpoints[i].y
        if is_need_cut:
            if trajectory.roadpoints[i].v ==0:
                safe_i_last = max([0, i-1])
                path[i, :] = path[safe_i_last, :]
    return path





def plot_scene_wp(wp_x, wp_y, ob):
    n_point, n_car= len(wp_x), len(wp_x[0])

    for i_ob in ob:
        theta = np.linspace(0, 2*pi, 200)
        x = i_ob[0] + 0.5*1 * np.cos(theta)
        y = i_ob[1] + 0.5*1 * np.sin(theta)
        plt.plot(x, y, 'k-')
    for j in range(n_point):
        for i in range(n_car):
            plt.plot(wp_x[j][i], wp_y[j][i], 'ro')

def visual():
    '''在正常坐标系下画图

    '''
    rospy.init_node('visual_multi_car', anonymous=True)

    
    rospy.loginfo("Simulation: map loaded")

    vehicle_state_list = []
    for i_car in range(n_car):
        vehicle_pose_state = [0,0,0]
        vehicle_state_list.append(vehicle_pose_state)


    for i_car in range(n_car):
        id = car_ids[i_car]
        # 输入控制量
        rospy.Subscriber('car'+str(id)+'/purepusuit/preview_point', Point, getPrewierPoint, i_car)
        rospy.Subscriber('car'+str(id)+'/local_trajectory', Trajectory, get_local_trajectory, i_car)
        rospy.Subscriber('car'+str(id)+'/global_trajectory', Trajectory, get_global_trajectory, i_car)
        rospy.Subscriber('car'+str(id)+'/gps', Odometry, sub_gps_states, i_car)

    rospy.Subscriber('/temp_goal', Trajectory, get_wp)

    rate = rospy.Rate(10)
    origin_shift = [-85, -7]

    wp_x = [0.0, 0.0, 0.0,  15.5, 15.499999999999998, 20.0, 40.0, 40.0, 40.0]
    wp_y = [0, -6.5, -3.8, 7.598076211353316, 2.401923788646685, 5.0, 8.0, 2.0, 5.0]
    wp_x = np.array(wp_x) +origin_shift[0]
    wp_y = np.array(wp_y) + origin_shift[1]

    n_wp = len(wp_x)
    
    while not rospy.is_shutdown():
        # plot simulation
        plt.clf()

        plt.plot(boundary[:,0], boundary[:,1], 'r-')

        for i_car in range(n_car):
            id = car_ids[i_car]

            if is_gps_ready_list[i_car]:
                vehicle_pose_state = pose_states[i_car]
                plt.plot(vehicle_pose_state.position.x, vehicle_pose_state.position.y, 'bo')
                draw_car(vehicle_pose_state.position.x, vehicle_pose_state.position.y, vehicle_pose_state.orientation.z/180*np.pi, 0)
            
            if  is_preview_point_ready_list[i_car]:
                previewPoint = preview_point_list[i_car]
                plt.plot(previewPoint.x, previewPoint.y, 'k*')
            
            if is_local_trajectory_ready_list[i_car]:
                path = get_path_xy(local_trajectory_list[i_car])
                plt.plot(path[:,0], path[:,1], 'g.')
            if is_global_trajectory_ready_list[i_car]:
                path = get_path_xy(global_trajectory_list[i_car], is_need_cut=False)
                plt.plot(path[:,0], path[:,1], 'k-')
        if is_wp_ready:
            wp_np = get_path_xy(formation_points_traj, is_need_cut=False)
            plt.plot(wp_np[:, 0], wp_np[:, 1], 'r*')

        # plt.show()
        plt.axis('square')

        plt.pause(0.001)

        rate.sleep()
    # plt.show()

if __name__ == '__main__':

    visual()
    # plt.figure(1)
    # draw_car(0,0,0,0)

    # plt.axis('square')
    # plt.show()
    # print('hold on')