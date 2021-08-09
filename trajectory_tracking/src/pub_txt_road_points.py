#!/usr/bin/env python
# -*- coding:utf8 -*-
'''
1. 采集得到路点后，能够直接回放txt中的路径
2. 发布自定义的
'''

from math import sqrt
import rospy
import numpy as np
from trajectory_tracking.msg import RoadPoint
from trajectory_tracking.msg import Trajectory


def talker(roadmap_path):
    pub = rospy.Publisher('local_trajectory', Trajectory, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    if len(roadmap_path) !=0:        # 有地图，加载地图文件
        data = np.loadtxt(roadmap_path, delimiter=',')
        print(roadmap_path)
    else:                                           # 没有地图，可以自定义编辑x,y
        data = np.zeros([500,5])
        x = np.linspace(0,5,500)
        r=5
        # y = (r**2 - (x-r)**2)**0.5        # 走圆弧
        y = -0.6                                                 # 走直线
        data[:, 0]=x
        data[:,1]=y

    local_trajectory = Trajectory()
    for i in range(len(data)):
        road_point = RoadPoint()
        road_point.x = data[i, 0]   # x
        road_point.y =  data[i, 1]   # y
        road_point.v = 1.5   # v=1.  匀速运动，速度设为1

        local_trajectory.roadpoints.append(road_point)

        
    while not rospy.is_shutdown():

        pub.publish(local_trajectory)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('gnss_txt_pub', anonymous=True)
    # 默认为0，如果不输入地图，则手动生成
    roadmap_path = rospy.get_param("/roadmap_path", 'line_cricle.txt')
    talker(roadmap_path)

