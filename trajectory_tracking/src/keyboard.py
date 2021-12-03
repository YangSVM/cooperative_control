#!/usr/bin/env python3
from threading import Thread
from pynput import keyboard
import rospy
import time


from std_msgs.msg import Int16MultiArray


rospy.init_node("keyboard_input")
rospy.loginfo("keyboard init")

control_flag = Int16MultiArray()
control_flag.data = [-1 for i in range(8)]



pub_flag = rospy.Publisher('control_flag',Int16MultiArray, queue_size=1)



def on_press(key):
    try:
        # print('\nalphanumeric key {0} pressed'.format(key.char))
        legal_inputs = [str(i) for i in range(8)]
        if key.char in legal_inputs:
            
            id = int(key.char)

            control_flag.data[id -1] = - control_flag.data[id -1]
            #print(control_flag.data)
            #pub_flag.publish(control_flag)

    except AttributeError:
        print('\nspecial key {0} pressed'.format(key))

def send_flag():
    while True:
        pub_flag.publish(control_flag)
        time.sleep(0.1)


t1=Thread(target=send_flag)
t1.start()


with keyboard.Listener(on_press=on_press) as listener:
    listener.join()




