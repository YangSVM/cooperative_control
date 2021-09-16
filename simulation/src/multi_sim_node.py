#!/usr/bin/env python3
'''
仿真的参考系用的右手系
v2.0 输入输出画图均采用右手系
'''
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
import math
from utils.config_control import *
import math
from enum import Enum
from geometry_msgs.msg import Point
from trajectory_tracking.msg import Trajectory
from math import pi

from formation_dec.formation_zoo import *
from formation_dec.config_formation import *


# 全局变量。
preview_point = Point(0,0,0)
local_trajectory = Trajectory()

preview_point_list = [Point(0,0,0) for i in range(n_car)]
local_trajectory_list = [Trajectory() for i in range(n_car)]
is_local_trajectory_ready_list = [False for i in range(n_car)]

global_trajectory_list = [Trajectory() for i in range(n_car)]
is_global_trajectory_ready_list = [False for i in range(n_car)]

wp_traj = Trajectory() 
is_wp_ready = False


class Gear(Enum):
    GEAR_DRIVE = 1
    GEAR_REVERSE = 2


class VehicleState:
    '''yaw 弧度制
    '''
    def __init__(self, x=0.0, y=0.0, yaw=0.0,
                 v=0.0, gear=Gear.GEAR_DRIVE):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.e_cg = 0.0
        self.theta_e = 0.0

        self.gear = gear
        self.steer = 0.0
        self.gps_msg = Odometry()

    def UpdateVehicleState(self, delta, a, e_cg, theta_e,
                           gear=Gear.GEAR_DRIVE):
        """
        update states of vehicle
        :param theta_e: yaw error to ref trajectory
        :param e_cg: lateral error to ref trajectory
        :param delta: steering angle [rad]
        :param a: acceleration [m / s^2]
        :param gear: gear mode [GEAR_DRIVE / GEAR/REVERSE]
        """

        wheelbase_ = wheelbase
        delta, a = self.RegulateInput(delta, a)

        self.steer = delta
        self.gear = gear
        self.x += self.v * math.cos(self.yaw) * ts
        self.y += self.v * math.sin(self.yaw) * ts
        self.yaw += - self.v / wheelbase_ * math.tan(delta) * ts
        self.e_cg = e_cg
        self.theta_e = theta_e

        if gear == Gear.GEAR_DRIVE:
            self.v += a * ts
        else:
            self.v += -1.0 * a * ts

        self.v = self.RegulateOutput(self.v)

    @staticmethod
    def RegulateInput(delta, a):
        """
        regulate delta to : - max_steer_angle ~ max_steer_angle
        regulate a to : - max_acceleration ~ max_acceleration
        :param delta: steering angle [rad]
        :param a: acceleration [m / s^2]
        :return: regulated delta and acceleration
        """

        if delta < -1.0 * max_steer_angle:
            delta = -1.0 * max_steer_angle

        if delta > 1.0 * max_steer_angle:
            delta = 1.0 * max_steer_angle

        if a < -1.0 * max_acceleration:
            a = -1.0 * max_acceleration

        if a > 1.0 * max_acceleration:
            a = 1.0 * max_acceleration

        return delta, a

    @staticmethod
    def RegulateOutput(v):
        """
        regulate v to : -max_speed ~ max_speed
        :param v: calculated speed [m / s]
        :return: regulated speed
        """

        max_speed_ = max_speed

        if v < -1.0 * max_speed_:
            v = -1.0 * max_speed_

        if v > 1.0 * max_speed_:
            v = 1.0 * max_speed_

        return v

    def GetGps(self):
        self.gps_msg.pose.pose.position.x = self.x
        self.gps_msg.pose.pose.position.y = self.y
        self.gps_msg.twist.twist.angular.z =  (self.yaw)/pi*180


        self.gps_msg.twist.twist.linear.x  = self.v*math.cos(math.radians(self.yaw))
        self.gps_msg.twist.twist.linear.y  = self.v*math.sin(math.radians(self.yaw))

        return self.gps_msg


def vehicle_update(msg, vehicleState):

    v = msg.data[0]/36
    steer =(msg.data[1]*30/1024 ) *np.pi/180
    v0 = vehicleState.v
    a = (v - v0)/ts
    vehicleState.UpdateVehicleState(steer, a, 0, 0)


def getPrewierPoint(msg, id):
    '''preview_point_list 已经在右手系了
    '''
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

def get_wp(msg):
    global wp_traj, is_wp_ready
    wp_traj =  msg
    is_wp_ready = True

def trajectory2np(trajectory):
    n_points = len(trajectory.roadpoints)

    path = np.zeros([n_points, 2])
    for i in range(n_points):
        path[i,0] = trajectory.roadpoints[i].x
        path[i,1] = trajectory.roadpoints[i].y
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

def simulation():
    '''在正常坐标系下画图

    '''
    rospy.init_node('simulation_multi_car', anonymous=True)

    # load road map
    # mapFilePath = rospy.get_param('roadmap_path')
    # map = np.loadtxt(mapFilePath)
    # 加载所有车辆的初始位置
    vehicle_state_origin = rospy.get_param('vehicle_state_origin', '')

    # state_map_origin = map[0, :]
    # x,y,yaw.右手系
    vehicle_state_list = []

    global task
    if task == 1:

        pos_line =formation_line(center_line[0, :], 0, n_car, d_car)
        state_map_origin = [2 ,17,-90]    
        # state_map_origin = [2 ,-8,-90]    # fast  test
        # state_map_origin = [-2, 8,-90]  # field test

        for i in range(n_car):
            # vehicleState = VehicleState(state_map_origin[0]-2*i, state_map_origin[1], state_map_origin[2] *np.pi /180 )
            vehicleState = VehicleState(pos_line[i, 0], pos_line[i, 1], -pi/2)
            vehicle_state_list.append(vehicleState)

    if task == 3:
        global battle_pos, battle_theta, battle_theta_norm
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

        for i in range(n_car):
            vehicleState = VehicleState(pos_start[i, 0], pos_start[i, 1], math.atan2(2, -1) )
            vehicle_state_list.append(vehicleState)
    


    for i in range(n_car):
        id = car_ids[i]
        # 输入控制量
        rospy.Subscriber('car'+str(id)+'/control_cmd',Int16MultiArray, vehicle_update, vehicle_state_list[i])
        rospy.Subscriber('car'+str(id)+'/purepusuit/preview_point', Point, getPrewierPoint, i)
        rospy.Subscriber('car'+str(id)+'/local_trajectory', Trajectory, get_local_trajectory, i)
        rospy.Subscriber('car'+str(id)+'/global_trajectory', Trajectory, get_global_trajectory, i)
        rospy.Subscriber('/temp_goal', Trajectory, get_wp)

    # 输出GPS坐标
    state_pubs = [rospy.Publisher('car'+str(id)+'/gps', Odometry, queue_size=1) for id in car_ids]

    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        # plot simulation
        # plt.cla()
        # plt.plot(map[:,2], map[:, 1], 'k--')
        # plot_scene_wp([], [], ob)
        # for i_ob in ob:
        #     theta = np.linspace(0, 2*pi, 200)
        #     x = i_ob[0] + 0.5*1 * np.cos(theta)
        #     y = i_ob[1] + 0.5*1 * np.sin(theta)
        #     plt.plot(x, y, 'k-')
        # plt.plot(ob[:, 0], ob[:,1], 'bo')
        # for i_wp in range(n_wp):
        #     plt.plot(wp_x[i_wp], wp_y[i_wp], 'r*')

        for i in range(n_car):
            id = car_ids[i]
            vehicleState = vehicle_state_list[i]
            gps_msg = vehicleState.GetGps()
            state_pubs[i].publish(gps_msg)

        #     plt.plot(boundary[:,0], boundary[:,1], 'r-')

        #     draw_car(vehicleState.x, vehicleState.y, vehicleState.yaw, vehicleState.steer)
        #     previewPoint = preview_point_list[i]
        #     if  abs (previewPoint.x) > (1e-5):
        #         plt.plot(previewPoint.x, previewPoint.y, 'r*')
            
        #     if is_local_trajectory_ready_list[i]:
        #         path = trajectory2np(local_trajectory_list[i])
        #         plt.plot(path[:,0], path[:,1], 'g.')
        #     if is_global_trajectory_ready_list[i]:
        #         path = trajectory2np(global_trajectory_list[i])
        #         plt.plot(path[:,0], path[:,1], 'k-')
        # if is_wp_ready:
        #     wp_np = trajectory2np(wp_traj)
        #     plt.plot(wp_np[:, 0], wp_np[:, 1], 'r*')

        # # plt.show()
        # plt.axis('square')
        # plt.pause(0.001)

        rate.sleep()
    # plt.show()

if __name__ == '__main__':
    # draw_car(0, 0, 0, 0.2)
    # plt.show()
    simulation()