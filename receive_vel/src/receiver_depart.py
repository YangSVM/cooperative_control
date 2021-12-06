#!/usr/bin/env python3

import rospy 

from  nav_msgs.msg  import Odometry
import math

n_car = 7

pose_lists = [[] for i in range(n_car)]


def msgCallback(data, i):
    global pose_lists
    pose_lists[i].append(data)
    

def divide():
    for i in range(n_car):
        record(pose_lists[i], i)    


def record(data, id):
    
    f = open("routes/scene_2_vel_3_true"+str(id)+".txt", "w")
    
    for item in data:
        v = math.hypot(item.twist.twist.linear.x, item.twist.twist.linear.y)
        f.write("0   "+ str('%.3f' %item.pose.pose.position.y)+ "   " + str('%.3f' %item.pose.pose.position.x) + "   " +"0.000   0   0.0000000   " + str('%.3f' %v))
        f.write("\n")

    f.close()

    

def receive():


    rospy.init_node("receive_vel", anonymous=True)
    
    for id in range(n_car):
        rospy.Subscriber('car'+str(id)+'/gps', Odometry, msgCallback, id)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rospy.spin()
        rate.sleep()

    divide()

    
if __name__ == '__main__':
    receive()
