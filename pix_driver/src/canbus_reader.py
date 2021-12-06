#!/usr/bin/env python
import rospy
from dbc import *
import can
from pix_driver.msg import pix_feedback

# parameters setup
dbc_path = rospy.get_param("dbc_path")
can_type = rospy.get_param("can_type")
can_channel = rospy.get_param("can_channel")

# dbc file setup
ms = decode_dbc(dbc_path)
m = ms.get_message_by_name('Auto_data_feedback')

# canbus setup
bus = can.interface.Bus(bustype=can_type, channel=can_channel, bitrate=500000)

def get_data():
    result = pix_feedback()
    bus.set_filters([{'can_id': 0x193, 'can_mask': 0x7FF, 'extended': False}])
    raw_data = bus.recv()
    feedback = m.decode(list(raw_data.data))

    result.header.stamp = rospy.Time.now()
    result.Speed_feedback = feedback['Speed_feedback']
    result.F_steer_feedback = feedback['F_steer_feedback']
    result.Braking_feedback = feedback['Braking_feedback']
    result.Gear_feedback = feedback['Gear_feedback']
    result.mode_feedback = feedback['mode_feedback']
    result.L_steer_light_feedback = feedback['L_steer_light_feedback']
    result.R_steer_light_feedback = feedback['R_steer_light_feedback']
    result.Tail_light_feedback = feedback['Tail_light_feedback']
    result.Braking_light_feedback = feedback['Braking_light_feedback']
    result.Vehicle_status_feedback = feedback['Vehicle_status_feedback']
    result.Vehicle_mode_feedback = feedback['Vehicle_mode_feedback']
    result.Emergency_stop_feedback = feedback['Emergency_stop_feedback']

    return result 

def feedback_talker():
    rospy.init_node('pix_chassis_feedback_node', anonymous=True)
    pub = rospy.Publisher('pix_chassis_feedback', pix_feedback, queue_size=1)
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
        pub.publish(get_data())
        r.sleep()


if __name__ == '__main__':
    feedback_talker()
