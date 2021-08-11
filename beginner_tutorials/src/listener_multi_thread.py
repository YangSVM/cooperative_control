#!/usr/bin/env python3
'''
python 收发测试
只需要rospy.Subscriber('car2/gps', Int32,  sub)。python会自动生成子线程完成调用。
'''
import rosgraph
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

def sub(msg):
    print(msg)

if __name__=='__main__':
    rospy.init_node('multi_test')
    rospy.Subscriber('car2/gps', Int32,  sub)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print('main_thread')
        rate.sleep()
