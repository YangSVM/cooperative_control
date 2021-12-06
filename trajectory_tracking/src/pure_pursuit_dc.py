#!/usr/bin/env python3
'''
dc: differential chassis: 差速底盘。
极创大车轨迹跟踪节点。
'''
import numpy as np
import rospy
from std_msgs.msg import  String
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point

from trajectory_tracking.msg import Trajectory
from math import sin,cos

previewPoint = Point(0,0,0)

# 配置参数：GPS安装偏移量
bias = [0.57, 0]

class PurePursuit():
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.error_ending = True   

        self.is_gps_ready = False
        self.is_local_trajectory_ready = False
        
        self.gps_msg = Odometry()
        self.local_traj = Trajectory()
        self.posture = np.zeros(3)           # x,y,yaw。x+正东,y+正北,yaw逆时针为正，正东为零，角度制
        self.vel = np.zeros(2)                      # 大地坐标系。

        self.preview_point = Point()

        # 输入 gnss, 规划的局部避撞轨迹
        rospy.Subscriber('gps', Odometry,  self.compute_cmd)
        rospy.Subscriber('local_trajectory', Trajectory, self.get_local_trajectory)

        # 输出控制指令
        self.pub_cmd = rospy.Publisher('control_cmd', String, queue_size=1)

        self.pub_preview_point= rospy.Publisher('purepusuit/preview_point',Point, queue_size=1)

        
        #  control_cmd initialize
        self.cmd_string = String()
        self.cmd = [0,0]
        # 控制频率可以稍低一些
        self.rate = rospy.Rate(50)          
    
    def run(self):

        while not rospy.is_shutdown():
            
            if not self.error_ending and self.is_local_trajectory_ready and self.is_gps_ready :
                self.cmd_string = str(self.cmd[0]) + ',' + str(self.cmd[1])
                self.pub_cmd.publish(self.cmd_string)
                self.pub_preview_point.publish(self.preview_point)
            self.rate.sleep()

    def compute_cmd(self, msg):
        self.error_ending = True
        self.is_gps_ready = True

        # 赋值
        self.gps_msg = msg
        self.posture[0] = (self.gps_msg.pose.pose.position.x)
        self.posture[1] = (self.gps_msg.pose.pose.position.y)

        self.posture[2] = (self.gps_msg.twist.twist.angular.z)
        yaw = self.posture[2]/180*np.pi


        new_bias_x=bias[0]*sin(yaw) + bias[1]*cos(yaw)
        new_bias_y=-bias[0]*cos(yaw) + bias[1]*sin(yaw)
        self.posture[0] = self.posture[0] + new_bias_x
        self.posture[1] = self.posture[1] + new_bias_y

        self.vel[0] = (self.gps_msg.twist.twist.linear.x)
        self.vel[1] = (self.gps_msg.twist.twist.linear.y)

        
        # if abs(self.gps_msg.twist.twist.linear.z - 42)>1e-3:
        #     return

        if not self.is_local_trajectory_ready:
            rospy.logwarn('waiting for local trajectory')
            return
        
        n_roadpoint  = len(self.local_traj.roadpoints)
        if n_roadpoint <= 0:
            # stop for None trajectory:
            self.cmd[0] = 0
            return

        local_traj_xy = np.zeros([n_roadpoint, 2])
        for i  in range(n_roadpoint):
            local_traj_xy[i, 0] = self.local_traj.roadpoints[i].x
            local_traj_xy[i, 1] = self.local_traj.roadpoints[i].y

        # local_traj_xy
        # 边界情况。收到空的局部轨迹，停车  
        if len(local_traj_xy) == 0:
            self.cmd[0] = 0
            self.cmd[1] = 0
            return

        # find the current waypoint according to distance.
        id_current, distance_current = self.get_current_roadpoint(local_traj_xy, self.posture)

        # 预瞄点太远直接停车
        if distance_current > 2 :
            self.cmd[0] = 0
            self.cmd[1] = 0
            return


        preview_distance = 1
        # find the preview roadpoint according to preview distance 
        id_preview, preview_distance_real =  self.get_preview_roadpoint(local_traj_xy, id_current, preview_distance, self.posture)
        yaw = self.posture[2]
        preview_x , preview_y = local_traj_xy[id_preview, 0],  local_traj_xy[id_preview, 1]

        self.preview_point.x, self.preview_point.y = local_traj_xy[id_preview, 0],  local_traj_xy[id_preview, 1]
        
        # delta_y：纯追踪算法中关键量。预瞄点 距离 车质心 车身右方侧向距离。
        delta_y =  (preview_x - self.posture[0]) * np.sin(yaw * np.pi / 180) - (preview_y - self.posture[1]) * np.cos(yaw * np.pi / 180)
        # delta_y = -(preview_x - self.posture[0]) * np.sin(yaw * np.pi / 180) +   (preview_y - self.posture[1]) * np.cos(yaw * np.pi / 180);


        # 算法公式。曲率 kappa = 2*y/ 距离平方。
        preview_curvature = 2* delta_y / (preview_distance_real**2 )


        vel_target  = self.local_traj.roadpoints[id_current].v

        vel_cmd = vel_target

        if vel_cmd ==0:
            print('stop!')

        self.cmd[0] = vel_cmd
        # 角速度右转为正
        self.cmd[1] = preview_curvature * self.cmd[0]

        # 预瞄距离过小直接停车
        if preview_distance_real < preview_distance/3:
            self.cmd[0] = 0
            self.cmd[1] = 0
        
        # 正常运行时，从此处完成
        self.error_ending = False
        return


    def get_current_roadpoint(self, local_traj_xy, posture):
        position = posture[:2]
        distance = np.linalg.norm(local_traj_xy - position, axis=1)
        index =  np.argmin(distance)
        min_distance = distance[index]

        index = np.where(np.abs(min_distance - distance)< 0.05)
        index = index[0][-1]
        if min_distance > 1:
            rospy.logwarn('too far from road. Distance to nearest waypoint: '+ str(min_distance))
        return index, min_distance

    def get_preview_roadpoint(self, local_traj_xy, id_current, preview_distance, posture):
        position = posture[:2]
        # distance： 实际与路点的距离 - 预瞄距离
        distance = np.linalg.norm(local_traj_xy[id_current:, :] - position, axis=1) - preview_distance
        # 距离自车最近点再增加1m
        # s = np.linalg.norm(local_traj_xy[1:, :] - local_traj_xy[:-1, :], axis=1)
        # s = np.cumsum(s)
        
        # distance = np.abs(s -1)


        id_preview = np.argmin(np.abs(distance))
        
        id_preview = id_preview + id_current
        
        preview_distance_error = distance[id_preview - id_current]
        # 如果实际距离-预瞄距离 过小
        if preview_distance_error < -0.5 :
            rospy.loginfo('not long enough for preview. preview the road end point instead')

        return id_preview, preview_distance_error + preview_distance


    def get_local_trajectory(self, msg):
        self.is_local_trajectory_ready = True
        self.local_traj = msg
       


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_differential_chassis')
    pure_pursuit = PurePursuit()
    pure_pursuit.run()
    # rospy.spin()
