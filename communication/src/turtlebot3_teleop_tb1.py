#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import numpy as np
import nav_msgs.msg
import geometry_msgs.msg
from rospy.numpy_msg import numpy_msg
from turtlebot3_teleop.msg import Floats
from tf.transformations import euler_from_quaternion
# 定义全局变量
x = 0
y = 0
w_o = 0
x_o = 0
y_o = 0
z_o = 0
yaw_t = 0
liner_speed = 0
angular_speed = 0
liner_speed_old = 0
angular_speed_old = 0
 
X_t = 0
Y_t = 0
X_t_Pre = 0
Y_t_Pre = 0
X_sim = [0.0 / 10, 0.0 / 10, 0.0 / 10]        # 目标点x坐标
Y_sim = [0.0 / 10, 0.0 / 10, 0.0 / 10]        # 目标点y坐标
r = 0
 
 
def Trans_robot_pose(msg):  # 回调函数
    # 位置坐标声明
    global x
    global y
    global w_o    # 当前小车位姿的四元数信息
    global x_o
    global y_o
    global z_o
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w_o = msg.pose.pose.orientation.w
    x_o = msg.pose.pose.orientation.x
    y_o = msg.pose.pose.orientation.y
    z_o = msg.pose.pose.orientation.z
    return w_o, y_o, z_o, x_o, x, y


def callback(data):
    a = data.data
    err = math.sqrt(math.pow((a[0] - X_sim[-1]), 2) + math.pow((a[1] - Y_sim[-1]), 2))
    if err > 0.01:
        X_sim.append(a[0])
        Y_sim.append(a[1])

 
if __name__ == '__main__':
    rospy.init_node('item1')
 
    turtle_vel = rospy.Publisher('/tb3_0/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    rate = rospy.Rate(10.0)
 
    while not rospy.is_shutdown():
        msg = geometry_msgs.msg.Twist()
        (roll, pitch, yaw) = euler_from_quaternion([x_o,y_o,z_o,w_o])  # 将四元数转化为roll, pitch, yaw
        if yaw < 0:
            yaw = yaw + 2 * math.pi
 
        X_t = X_sim[r]
        Y_t = Y_sim[r]
 
        D_err = math.sqrt(math.pow((X_t - x), 2) + math.pow((Y_t - y), 2))
        # 判断坐标象限
        if (Y_t - y) == 0 and (X_t - x) > 0:
            yaw_t = 0
        if (Y_t - y) > 0 and (X_t - x) > 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x))
        if (Y_t - y) > 0 and (X_t - x) == 0:
            yaw_t = 0.5 * math.pi
        if (Y_t - y) > 0 and (X_t - x) < 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + math.pi
        if (Y_t - y) == 0 and (X_t - x) < 0:
            yaw_t = math.pi
        if (Y_t - y) < 0 and (X_t - x) < 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + math.pi
        if (Y_t - y) < 0 and (X_t - x) == 0:
            yaw_t = 1.5 * math.pi
        if (Y_t - y) < 0 and (X_t - x) > 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + 2 * math.pi
 
        Theta_err = yaw_t - yaw
 
        if Theta_err < -math.pi:
            Theta_err = Theta_err + 2 * math.pi
        if Theta_err > math.pi:
            Theta_err = Theta_err - 2 * math.pi
 
        if D_err < 0.1:        # 当距当前点小于一定值的时候去下一个点
            X_t_Pre = X_t
            Y_t_Pre = Y_t
            r = r + 1
            print r
        else:
            # 仍然先只使用最简单的比例控制
            liner_speed = 0.2 * D_err
            angular_speed = 0.2 * Theta_err
 
            msg.linear.x = liner_speed
            msg.angular.z = angular_speed
 
            liner_speed_old = liner_speed
            angular_speed_old = angular_speed
            turtle_vel.publish(msg)    # 发布运动指令

        rospy.Subscriber('/tb3_0/odom', nav_msgs.msg.Odometry,  Trans_robot_pose) # 获取位姿信息
	rospy.Subscriber('floats', Floats, callback, queue_size=1)
 
        rate.sleep()
    rospy.spin()
