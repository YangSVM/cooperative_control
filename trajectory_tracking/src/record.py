#!/usr/bin/env python3
'''
修改为右手系
'''
import numpy as np
import rospy
from nav_msgs.msg import Odometry


class Record():
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.f_name = 'record.txt'
        rospy.Subscriber('gps', Odometry,  self.record_gps)


    def record_gps(self, msg):
        posture = -1*np.ones([1,4]) 
        posture[0,0] = (msg.pose.pose.position.x)
        posture[0,1] = (msg.pose.pose.position.y)
        posture[0,2] = (msg.twist.twist.angular.z) 
        posture[0,3] = (msg.twist.twist.linear.z)       # gnss status
        with open(self.f_name,'a') as f:
            np.savetxt(f, posture, delimiter=',', fmt='%.2f')


if __name__ == '__main__':
    rospy.init_node('record_gnss', anonymous=True)
    record = Record()
    rospy.spin()
