#!/usr/bin/env python3
'''
--- 未完成
--- gazebo 有模型方法可能难以运用
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[1, 0.0, 0.0]' '[0.0, 0.0, 1]'
转弯半径不为1。可能使用PID算法能够解决
'''

import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
import math
import os
import sys
import math
from enum import Enum
from geometry_msgs.msg import Point

from nav_msgs.msg import Path
from trajectory_tracking.msg import RoadPoint
from trajectory_tracking.msg import Trajectory
from geometry_msgs.msg import Twist

previewPoint = Point(0,0,0)

# Quaternion to yaw
def  quart_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw



class PurePursuit():
    def __init__(self):

        self.is_gps_ready = False
        self.is_local_trajectory_ready = False
        
        self.gps_msg = Odometry()
        self.local_traj = Trajectory()
        self.posture = np.zeros(3)           # x,y,yaw
        self.vel = np.zeros(2)                      # 车辆坐标系

        self.preview_point = Point()

        # 输入 gnss, 规划的局部避撞轨迹
        rospy.Subscriber('odom', Odometry,  self.compute_cmd)
        rospy.Subscriber('local_trajectory', Trajectory, self.get_local_trajectory)

        # 输出控制指令
        self.pub_cmd = rospy.Publisher('cmd_vel',Twist, queue_size=1)

        self.pub_preview_point= rospy.Publisher('purepusuit/preview_point',Point, queue_size=1)

        
        #  control_cmd initialize
        self.cmd = Int16MultiArray()
        self.cmd_twist = Twist()
        self.cmd.data = np.zeros([13], dtype=int).tolist()
        self.cmd.data[3] , self.cmd.data[8], self.cmd.data[9]=1, 1, 1
        self.rate = rospy.Rate(60)
    
    def run(self):

        while not rospy.is_shutdown():
            
            if self.is_local_trajectory_ready and self.is_gps_ready :
                self.pub_cmd.publish(self.cmd_twist)
                self.pub_preview_point.publish(self.preview_point)
            self.rate.sleep()

    def compute_cmd(self, msg):
        self.is_gps_ready = True

        # 赋值
        self.gps_msg = msg
        self.posture[0] = (self.gps_msg.pose.pose.position.x)
        self.posture[1] = (self.gps_msg.pose.pose.position.y)
        # self.posture[2] = (self.gps_msg.twist.twist.angular.z) 
        _, _, yaw_odom = quart_to_rpy(self.gps_msg.pose.pose.orientation)
        self.posture[2] = yaw_odom
        self.vel[0] = (self.gps_msg.twist.twist.linear.x)
        self.vel[1] = (self.gps_msg.twist.twist.linear.y)

        # 以下算法适用于左手系，需要修改x,y, yaw
        self.posture[0] = (self.gps_msg.pose.pose.position.y)
        self.posture[1] = (self.gps_msg.pose.pose.position.x)
        self.posture[2] = (-yaw_odom + math.pi/2)*180/math.pi


        if not self.is_local_trajectory_ready:
            rospy.logwarn('waiting for local trajectory')
            return
        
        n_roadpoint  = len(self.local_traj.roadpoints)
        local_traj_xy = np.zeros([n_roadpoint, 2])
        for i  in range(n_roadpoint):
            local_traj_xy[i, 1] = self.local_traj.roadpoints[i].x
            local_traj_xy[i, 0] = self.local_traj.roadpoints[i].y

        # find the current waypoint according to distance.
        id_current, distance_current = self.get_current_roadpoint(local_traj_xy, self.posture)

        preview_distance = 0.2
        # find the preview roadpoint according to preview distance 
        id_preview, preview_distance_real =  self.get_preview_roadpoint(local_traj_xy, id_current, preview_distance, self.posture)
        yaw = self.posture[2]
        preview_x , preview_y = local_traj_xy[id_preview, 0],  local_traj_xy[id_preview, 1]
 
        self.preview_point.x, self.preview_point.y = local_traj_xy[id_preview, 0],  local_traj_xy[id_preview, 1]
        
        # delta_y 为自车坐标系
        delta_y =  -(preview_x - self.posture[0]) * np.sin(yaw * np.pi / 180) +  (preview_y - self.posture[1]) * np.cos(yaw * np.pi / 180)

        preview_curvature = 2* delta_y / (preview_distance_real**2 )

        # 0.8 轴距
        angle = 0.8*preview_curvature*180/np.pi

        if np.abs(angle) >30:
            angle = np.sign(angle) *30
        self.cmd.data[1] = int(angle * 1024/30)

        vel_target  = self.local_traj.roadpoints[id_current].v
        vel_current = np.linalg.norm(self.vel)
        if vel_target > vel_current +0.2:
            vel_cmd = vel_current +0.2
        elif vel_target < vel_current +0.2:
            vel_cmd = vel_current -0.2
        else:
            vel_cmd = vel_target
        
        # 反向时，换倒车档
        if vel_cmd<0:
            vel_cmd = abs(vel_cmd)
            self.cmd.data[3] = 3

        self.cmd.data[0] = int (vel_cmd*36 )
        if preview_distance_real < 0.5:
            self.cmd.data[0] = 0
            self.cmd.data[1] = 0
        
        # 新加的发布twist用于仿真
        # heading：速度超橡胶
        heading = yaw_odom-(angle/2) * math.pi/180

        # turtlebot3的控制仅通过速度绝对值大小，角速度进行控制
        self.cmd_twist.linear.x = 0.2
        # self.cmd_twist.linear.y = vel_cmd*math.sin(heading)
        self.cmd_twist.angular.z = -vel_current*preview_curvature*10

        return


    def get_current_roadpoint(self, local_traj_xy, posture):
        position = posture[:2]
        distance = np.linalg.norm(local_traj_xy - position, axis=1)
        index =  np.argmin(distance)
        min_distance = distance[index]

        index = np.where(np.abs(min_distance - distance)< 0.05)
        index = index[0][-1]
        if min_distance > 1:
            rospy.logwarn('too far from road'+ str(min_distance))
        return index, min_distance

    def get_preview_roadpoint(self, local_traj_xy, id_current, preview_distance, posture):
        position = posture[:2]
        distance = np.linalg.norm(local_traj_xy[id_current:, :] - position, axis=1) - preview_distance
        id_preview = np.argmin(np.abs(distance))
        
        id_preview = id_preview + id_current
        
        preview_distance_error = distance[id_preview - id_current]
        if preview_distance_error > 0.05:
            rospy.loginfo('not long enough for preview. preview the road end point instead')

        return id_preview, preview_distance_error + 1


    def get_local_trajectory(self, msg):
        if (self.is_local_trajectory_ready):        # need to add flag
            return
        self.is_local_trajectory_ready = True
        self.local_traj = msg
       


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_py', anonymous=True)
    pure_pursuit = PurePursuit()
    pure_pursuit.run()
    # rospy.spin()
