#!/usr/bin/env python3

import rospy 

from trajectory_tracking.msg import  Trajectory, RoadPoint
from  nav_msgs.msg  import Odometry
import message_filters
import math

global pose_list, pose_list_1, pose_list_2, pose_list_3, pose_list_4, pose_list_5, pose_list_6, pose_list_7 
pose_list_1, pose_list_2, pose_list_3, pose_list_4, pose_list_5, pose_list_6, pose_list_7 = [], [], [],  [], [], [], []

# def msgCallback(data1, data2, data3, data4, data5, data6, data7):
#     global pose_list
#     pose_list = [data1.roadpoints, data2.roadpoints, data3.roadpoints, data4.roadpoints, \
#         data5.roadpoints,  data6.roadpoints, data7.roadpoints]
#     divide()

def msgCallback(data1, data2, data3, data4, data5, data6, data7):
    print('receive data')
    global pose_list, pose_list_1, pose_list_2, pose_list_3, pose_list_4, pose_list_5, pose_list_6, pose_list_7 
    pose_list_1.append(data1)
    pose_list_2.append(data2)
    pose_list_3.append(data3)
    pose_list_4.append(data4)
    pose_list_5.append(data5)
    pose_list_6.append(data6)
    pose_list_7.append(data7)
    

def divide():
    global pose_list
    pose_list = [pose_list_1, pose_list_2, pose_list_3, pose_list_4, pose_list_5, pose_list_6, pose_list_7]
    for i in range(7):
        record(pose_list[i], i)    


def record(data, id):
    
    f = open("/home/tiecun/catkin_ws/src/MA-L5-THICV/trajectory_tracking/receive_vel/src/routes/scene_7_vel_2_"+str(id)+".txt", "w")
    
    for item in data:
        v = math.hypot(item.twist.twist.linear.x, item.twist.twist.linear.y)
        f.write("0   "+ str('%.3f' %item.pose.pose.position.y)+ "   " + str('%.3f' %item.pose.pose.position.x) + "   " +"0.000   0   0.0000000   " + str('%.3f' %v))
        f.write("\n")

    f.close()

    

def receive():


    rospy.init_node("receive_vel", anonymous=True)
    
    # car1 = message_filters.Subscriber("/car1/global_trajectory", Trajectory)
    # car2 = message_filters.Subscriber("/car2/global_trajectory", Trajectory)
    # car3 = message_filters.Subscriber("/car3/global_trajectory", Trajectory)
    # car4 = message_filters.Subscriber("/car4/global_trajectory", Trajectory)
    # car5 = message_filters.Subscriber("/car5/global_trajectory", Trajectory)
    # car6 = message_filters.Subscriber("/car6/global_trajectory", Trajectory)
    # car7 = message_filters.Subscriber("/car7/global_trajectory", Trajectory)

    car1 = message_filters.Subscriber("/car1/gps", Odometry)
    car2 = message_filters.Subscriber("/car2/gps", Odometry)
    car3 = message_filters.Subscriber("/car3/gps", Odometry)
    car4 = message_filters.Subscriber("/car4/gps", Odometry)
    car5 = message_filters.Subscriber("/car5/gps", Odometry)
    car6 = message_filters.Subscriber("/car6/gps", Odometry)
    car7 = message_filters.Subscriber("/car7/gps", Odometry)

    carn = message_filters.ApproximateTimeSynchronizer([car1, car2, car3, car4, car5, car6, car7], 10, 1, allow_headerless = True)
    carn.registerCallback(msgCallback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # else:
        #     print("empty input data")
        rospy.spin()
        rate.sleep()

    divide()

    


if __name__ == '__main__':
    receive()
