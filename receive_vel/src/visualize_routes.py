#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

def visual():
    route = np.loadtxt("/home/tiecun/catkin_ws/src/MA-L5-THICV/trajectory_tracking/receive_vel/src/routes/scene_7_vel_2_1204_0.txt")
    plt.plot(route[:, 2], route[:, 1], 'k-')
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    visual()
