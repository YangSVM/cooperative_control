#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
import std_msgs.msg
from dbc import decode_dbc
import can


dbc_path = rospy.get_param("dbc_path")
can_type = rospy.get_param("can_type")
can_channel = rospy.get_param("can_channel")

ms = decode_dbc(dbc_path)
m = ms.get_message_by_name('Auto_control')
code = m.encode({'Steering': 0})

bus = can.interface.Bus(bustype=can_type, channel=can_channel, bitrate=500000)


pub = rospy.Publisher('canbus_message', Int16MultiArray, queue_size=1)

def callback(data):
    try:
        speed = data.data[0]
        if speed < 0:
            speed = 0
        can_message = m.encode({
            'Speed': speed,
            'Steering': data.data[1],
            'Braking': data.data[2],
            'Gear_shift': data.data[3],
            'EPB': data.data[4],
            'right_light': data.data[5],
            'left_light': data.data[6],
            'Front_light': data.data[7],
            'self_driving_enable': data.data[8],
            'Speed_mode': data.data[9],
            'Advanced_mode': data.data[10],
            'mode_selection': data.data[11],
            'State_control': data.data[12]
            })
        can_message = [387] + can_message

    except:
        can_message = [387] + 8*[0]
    send_message = can.Message(arbitration_id=387, data=can_message[1:], is_extended_id=False)
    bus.send(send_message)
    message = Int16MultiArray(data=can_message)
    rospy.loginfo(message.data)
    pub.publish(message)



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("control_cmd", Int16MultiArray, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
