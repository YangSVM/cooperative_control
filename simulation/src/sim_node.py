#!/usr/bin/env python3
import matplotlib.pyplot as plt
from utils.draw_lqr import draw_car
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
import math
from utils.config_control import *
import os
import sys
import math
from enum import Enum
from geometry_msgs.msg import Point
from trajectory_tracking.msg import Trajectory



from nav_msgs.msg import Path

previewPoint = Point(0,0,0)
# local_trajectory = Path()
local_trajectory = Trajectory()

is_local_trajectory_ready = False

class Gear(Enum):
    GEAR_DRIVE = 1
    GEAR_REVERSE = 2


class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0,
                 v=0.0, gear=Gear.GEAR_DRIVE):
                #  '''国际单位制'''
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
        self.gps_msg.pose.pose.position.x = self.y
        self.gps_msg.pose.pose.position.y = self.x
        self.gps_msg.twist.twist.angular.z =  (- self.yaw + np.pi/2) * 180/np.pi

        self.gps_msg.twist.twist.linear.x  = self.v*math.cos(math.radians(self.yaw))
        self.gps_msg.twist.twist.linear.y  = self.v*math.sin(math.radians(self.yaw))

        return self.gps_msg


def vehicle_update(msg, vehicleState):
    v = msg.data[0]/36
    steer =(msg.data[1]*30/1024 ) *np.pi/180
    v0 = vehicleState.v
    a = (v - v0)/ts
    vehicleState.UpdateVehicleState(steer, a, 0, 0)


def getPrewierPoint(msg):
    previewPoint.x = msg.y
    previewPoint.y = msg.x
    previewPoint.z = msg.z


def get_local_trajectory(msg):
    global local_trajectory, is_local_trajectory_ready
    local_trajectory = msg
    is_local_trajectory_ready = True


def get_path_xy(local_trajectory):
    n_points = len(local_trajectory.roadpoints)

    path = np.zeros([n_points, 2])
    for i in range(n_points):
        # path[i,0] = local_trajectory.poses[i].pose.position.x
        # path[i,1] = local_trajectory.poses[i].pose.position.y
        path[i,0] = local_trajectory.roadpoints[i].x
        path[i,1] = local_trajectory.roadpoints[i].y
    return path


def simulation():
    rospy.init_node('simulation', anonymous=True)

    # load road map
    # mapFilePath = rospy.get_param('roadmap_path')
    # map = np.loadtxt(mapFilePath)
    # state_map_origin = map[0, :]


    # rospy.loginfo("Simulation: map loaded")

    # vehicleState = VehicleState(state_map_origin[2], state_map_origin[1], -state_map_origin[3] *np.pi /180 + np.pi/2)

    vehicleState = VehicleState(-10, 0, -state_map_origin[3] *np.pi /180 + np.pi/2)
    # 输入控制量
    rospy.Subscriber('control_cmd',Int16MultiArray, vehicle_update, vehicleState)
    rospy.Subscriber('purepusuit/preview_point', Point, getPrewierPoint)
    # rospy.Subscriber('local_trajectory', Path, get_local_trajectory)

    rospy.Subscriber('local_trajectory', Trajectory, get_local_trajectory)



    # 输出GPS坐标
    state_pub = rospy.Publisher('gps', Odometry, queue_size=1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        gps_msg = vehicleState.GetGps()
        state_pub.publish(gps_msg)
    # plot simulation
        plt.cla()
        plt.plot(map[:,2], map[:, 1], 'k--')
        draw_car(vehicleState.x, vehicleState.y, vehicleState.yaw, vehicleState.steer)
        if  abs (previewPoint.x) > (1e-5):
            plt.plot(previewPoint.x, previewPoint.y, 'r*')
        if is_local_trajectory_ready:
            path = get_path_xy(local_trajectory)
            plt.plot(path[:,1], path[:,0], 'b-')
        # plt.show()
        plt.pause(0.001)

        rate.sleep()
    plt.show()

if __name__ == '__main__':
    # draw_car(0, 0, 0, 0.2)
    # plt.show()
    simulation()