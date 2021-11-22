#!/usr/bin/env python
# -*- coding:utf8 -*-
'''
1. 采集得到路点后，能够直接回放txt中的路径
2. 发布自定义的
'''

from math import sqrt
import time
import rospy
import numpy as np
from trajectory_tracking.msg import RoadPoint
from trajectory_tracking.msg import Trajectory


def talker(roadmap_path):
    pub = rospy.Publisher('car1/local_trajectory', Trajectory, queue_size=1)
    rate = rospy.Rate(10) # 10hz

    time1 = time.time()
    while not rospy.is_shutdown():
        if len(roadmap_path) !=0:        # 有地图，加载地图文件
            data = np.loadtxt(roadmap_path, delimiter=',')
            print(roadmap_path)
        else:                                           # 没有地图，可以自定义编辑x,y
            data = np.zeros([100,2])
            
            # y1 = np.linspace(-10,0,50)
            # x1 = np.zeros(50)
            # theta = np.linspace(0, 180, 50)*np.pi/180
            # r = 5
            # x2 = -r + r*np.cos(theta)
            # y2 = r*np.sin(theta)
            
            # data[:50, 0]=0
            # data[50:, 0]=x2
            # data[:50, 1]=y1
            # data[50:, 1]=y2

            data[:, 0]=8
            data[:, 1] = np.linspace(16,-16,100)

        local_trajectory = Trajectory()
        time2 = time.time()
        dt = time2-time1
        t_thre = 10
        if dt > t_thre:
            print('get up')
        for i in range(len(data)):
        
            road_point = RoadPoint()
            road_point.code = 0
            road_point.x = data[i, 0]   # x
            road_point.y =  data[i, 1]   # y
            if dt>t_thre:
                road_point.v = 1.5   # m/s  匀速运动
            else:
                road_point.v = 0    # 前5秒速度设为1
                print('here')   

            local_trajectory.roadpoints.append(road_point)

        
 

        pub.publish(local_trajectory)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('gnss_txt_pub', anonymous=True)
    # 默认为0，如果不输入地图，则手动生成
    # roadmap_path = rospy.get_param("/roadmap_path", 'line_cricle.txt')
    roadmap_path = rospy.get_param("/roadmap_path", '')

    talker(roadmap_path)

