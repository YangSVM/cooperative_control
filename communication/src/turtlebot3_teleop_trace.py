#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import math
import numpy as np
from rospy.numpy_msg import numpy_msg
from turtlebot3_teleop.msg import Floats

field = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

# initiate the search map
searchMap = np.zeros((100, 100))
n_X = len(searchMap)
n_Y = len(searchMap[0])
N = 0
for i in range(n_X):
    for j in range(n_Y):
        if field[i // (n_X // 20)][j // (n_Y // 20)] == 0:
            searchMap[i][j] = 255
            N += 255 * 9
        else:
            searchMap[i][j] = -1
            N += -1 * 9
n_X *= 3
n_Y *= 3

# initiate the robots, targets and paths
robot = [[0.0, 0.0, 60 * math.pi / 180], [10.0, 0.0, 30 * math.pi / 180]]
robot = np.array(robot)
target = []
n_R = len(robot)
n_T = len(target)
robotPath = [[] for i in range(n_R)]
targetFound = [0 for i in range(n_T + 1)]
for r in range(n_R):
    robotPath[r].append([robot[r][0], robot[r][1]])

# thermodynamics
diff = 0.1      # diffusion coefficient
diffN = 0       # diffusion time
s_0 = 2 * math.log(255)     # zero-point entropy
obstacle = -0.02

# mainloop
time = 0
velocity = 4
steel = 50 * math.pi / 180
deltaV = 4 - 4.58 * 2 * math.sin(0.5 * steel)
detectLength = 30
pathBuffer = [[-1, -1, -1] for i in range(n_R)]


if __name__ == '__main__':
    rospy.init_node('trace')

    pub = rospy.Publisher('floats', numpy_msg(Floats), queue_size=10)
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        print(time)
        print(N)
        print(robot)
        print(target)
        print(pathBuffer)
        # detection
        for r in range(n_R):
            detectAngle = [0, 0, 0, 0, 0, 0]
            mapBuffer = searchMap.copy()
            N = N - mapBuffer[round(robot[r][0]) // 3][round(robot[r][1]) // 3] + 1
            searchMap[round(robot[r][0]) // 3][round(robot[r][1]) // 3] += 1 / 9 - mapBuffer[round(robot[r][0]) // 3][
                round(robot[r][1]) // 3] / 9
            for i in range(1, detectLength + 1):
                for j in range(-detectLength // 2, detectLength // 2 + 1):
                    x = round(robot[r][0] + i * math.cos(robot[r][2]) - j * math.sin(robot[r][2]))
                    y = round(robot[r][1] + j * math.cos(robot[r][2]) + i * math.sin(robot[r][2]))
                    if x < 0 or x > n_X - 1 or y < 0 or y > n_Y - 1:
                        continue
                    norm = np.sqrt(np.abs(i ** 2 + j ** 2))
                    angle = math.atan(j / i) * 180 / math.pi
                    zone = math.floor((angle + 30) / 10)
                    if -1 < zone < 6 and norm < detectLength:
                        if detectAngle[zone] == 0:
                            for t in range(n_T):
                                if [x, y] == [round(target[t][0]), round(target[t][1])]:
                                    targetFound[t] = 1
                                    break
                            if mapBuffer[x // 3][y // 3] != -1:
                                N = N - mapBuffer[x // 3][y // 3] + 1
                                searchMap[x // 3][y // 3] += 1 / 9 - mapBuffer[x // 3][y // 3] / 9
                            else:
                                detectAngle[zone] = 1
                if max(targetFound) == 1:
                    break
            if max(targetFound) == 1:
                break
        if min(targetFound) == 1:
            break
        # renew the search map
        for d in range(diffN):
            mapBuffer = searchMap.copy()
            for i in range(n_X // 3):
                for j in range(n_Y // 3):
                    if searchMap[i][j] != -1:
                        for m in range(i - 1, i + 2):
                            for n in range(j - 1, j + 2):
                                if m < 0 or m > n_X // 3 - 1 or n < 0 or n > n_Y // 3 - 1:
                                    continue
                                if searchMap[m][n] == -1:
                                    continue
                                searchMap[i][j] = searchMap[i][j] + diff * (mapBuffer[m][n] - mapBuffer[i][j])
        # search decision
        if time % 3 == 0:
            pathBuffer = [[-1, -1, -1] for i in range(n_R)]
            decision = [0 for i in range(n_R)]
            for r1 in range(n_R - 1):
                for r2 in range(r1 + 1, n_R):
                    if np.sqrt(np.abs((robot[r1][0] - robot[r2][0]) ** 2 + (
                            robot[r1][1] - robot[r2][1]) ** 2)) < 4 * velocity:
                        J_m = -9999999999
                        if 0.5 * math.pi < np.abs(robot[r1][2] - robot[r2][2]) % math.pi < 1.5 * math.pi:
                            reverse = 1
                        else:
                            reverse = -1
                        for k1 in [-1, 0, 1]:
                            for k2 in [-1, 0, 1]:
                                for k3 in [-1, 0, 1]:
                                    J = 0
                                    mapBuffer = searchMap.copy()
                                    rr = [r1, r2]
                                    exitFlag = 0
                                    for r in range(2):
                                        robotBuffer = robot[rr[r]].copy()
                                        for k in [k1, k2, k3]:
                                            robotBuffer[2] = robotBuffer[2] + k * steel * (reverse ^ r)
                                            robotBuffer[0] = robotBuffer[0] + (
                                                        velocity - deltaV * np.abs(k)) * math.cos(
                                                robotBuffer[2] - 0.5 * k * steel * (reverse ^ r))
                                            robotBuffer[1] = robotBuffer[1] + (
                                                        velocity - deltaV * np.abs(k)) * math.sin(
                                                robotBuffer[2] - 0.5 * k * steel * (reverse ^ r))
                                            if robotBuffer[0] < 0 or robotBuffer[0] > n_X - 1 or robotBuffer[1] < 0 or \
                                                    robotBuffer[1] > n_Y - 1:
                                                exitFlag = 1
                                                break
                                            if searchMap[round(robotBuffer[0]) // 3][round(robotBuffer[1]) // 3] == -1:
                                                exitFlag = 1
                                                break
                                            detectAngle = [0, 0, 0, 0, 0, 0]
                                            mapBuffer2 = mapBuffer.copy()
                                            mapBuffer[round(robotBuffer[0]) // 3][round(robotBuffer[1]) // 3] += 1 / 9 - \
                                                                                                                 mapBuffer2[
                                                                                                                     round(
                                                                                                                         robotBuffer[
                                                                                                                             0]) // 3][
                                                                                                                     round(
                                                                                                                         robotBuffer[
                                                                                                                             1]) // 3] / 9
                                            for i in range(1, detectLength + 1):
                                                for j in range(-detectLength // 2, detectLength // 2 + 1):
                                                    x = round(
                                                        robotBuffer[0] + i * math.cos(robotBuffer[2]) - j * math.sin(
                                                            robotBuffer[2]))
                                                    y = round(
                                                        robotBuffer[1] + j * math.cos(robotBuffer[2]) + i * math.sin(
                                                            robotBuffer[2]))
                                                    norm = np.sqrt(np.abs(i ** 2 + j ** 2))
                                                    angle = math.atan(j / i) * 180 / math.pi
                                                    zone = math.floor((angle + 30) / 10)
                                                    if -1 < zone < 6 and norm < detectLength:
                                                        if x < 0 or x > n_X - 1 or y < 0 or y > n_Y - 1:
                                                            J = J + 0 * (s_0 - math.log(N / n_X / n_Y)) * N / n_X / n_Y
                                                            continue
                                                        if mapBuffer2[x // 3][y // 3] != -1:
                                                            if detectAngle[zone] == 0:
                                                                J = J + (s_0 - math.log(
                                                                    np.abs(mapBuffer2[x // 3][y // 3]) + 0.01)) * \
                                                                    mapBuffer2[x // 3][y // 3]
                                                                mapBuffer[x // 3][y // 3] += 1 / 9 - mapBuffer2[x // 3][
                                                                    y // 3] / 9
                                                            else:
                                                                J = J + obstacle * (
                                                                        s_0 - math.log(N / n_X / n_Y)) * N / n_X / n_Y
                                                        else:
                                                            detectAngle[zone] = 1
                                                            J = J + obstacle * (
                                                                        s_0 - math.log(N / n_X / n_Y)) * N / n_X / n_Y
                                        if exitFlag:
                                            break
                                    if exitFlag:
                                        continue
                                    if J > J_m:
                                        J_m = J
                                        pathBuffer[r1] = [k1, k2, k3]
                                        pathBuffer[r2] = [-k1, -k2, -k3]
                        if J_m == -9999999999:
                            continue
                        else:
                            decision[r1] = 1
                            decision[r2] = 1
            for r in range(n_R):
                if decision[r] == 0:
                    J_m = -9999999999
                    for k1 in [-1, 0, 1]:
                        for k2 in [-1, 0, 1]:
                            for k3 in [-1, 0, 1]:
                                J = 0
                                mapBuffer = searchMap.copy()
                                exitFlag = 0
                                robotBuffer = robot[r].copy()
                                for k in [k1, k2, k3]:
                                    robotBuffer[2] = robotBuffer[2] + k * steel
                                    robotBuffer[0] = robotBuffer[0] + (velocity - deltaV * np.abs(k)) * math.cos(
                                        robotBuffer[2] - 0.5 * k * steel)
                                    robotBuffer[1] = robotBuffer[1] + (velocity - deltaV * np.abs(k)) * math.sin(
                                        robotBuffer[2] - 0.5 * k * steel)
                                    if robotBuffer[0] < 0 or robotBuffer[0] > n_X - 1 or robotBuffer[1] < 0 or \
                                            robotBuffer[
                                                1] > n_Y - 1:
                                        exitFlag = 1
                                        break
                                    if searchMap[round(robotBuffer[0]) // 3][round(robotBuffer[1]) // 3] == -1:
                                        exitFlag = 1
                                        break
                                    detectAngle = [0, 0, 0, 0, 0, 0]
                                    mapBuffer2 = mapBuffer.copy()
                                    mapBuffer[round(robotBuffer[0]) // 3][round(robotBuffer[1]) // 3] += 1 / 9 - \
                                                                                                         mapBuffer2[
                                                                                                             round(
                                                                                                                 robotBuffer[
                                                                                                                     0]) // 3][
                                                                                                             round(
                                                                                                                 robotBuffer[
                                                                                                                     1]) // 3] / 9
                                    for i in range(1, detectLength + 1):
                                        for j in range(-detectLength // 2, detectLength // 2 + 1):
                                            x = round(
                                                robotBuffer[0] + i * math.cos(robotBuffer[2]) - j * math.sin(
                                                    robotBuffer[2]))
                                            y = round(
                                                robotBuffer[1] + j * math.cos(robotBuffer[2]) + i * math.sin(
                                                    robotBuffer[2]))
                                            norm = np.sqrt(np.abs(i ** 2 + j ** 2))
                                            angle = math.atan(j / i) * 180 / math.pi
                                            zone = math.floor((angle + 30) / 10)
                                            if -1 < zone < 6 and norm < detectLength:
                                                if x < 0 or x > n_X - 1 or y < 0 or y > n_Y - 1:
                                                    J = J + 0 * (s_0 - math.log(N / n_X / n_Y)) * N / n_X / n_Y
                                                    continue
                                                if mapBuffer2[x // 3][y // 3] != -1:
                                                    if detectAngle[zone] == 0:
                                                        J = J + (s_0 - math.log(
                                                            np.abs(mapBuffer2[x // 3][y // 3]) + 0.01)) * \
                                                            mapBuffer2[x // 3][y // 3]
                                                        mapBuffer[x // 3][y // 3] += 1 / 9 - mapBuffer2[x // 3][
                                                            y // 3] / 9
                                                    else:
                                                        J = J + obstacle * (
                                                                    s_0 - math.log(N / n_X / n_Y)) * N / n_X / n_Y
                                                else:
                                                    detectAngle[zone] = 1
                                                    J = J + obstacle * (s_0 - math.log(N / n_X / n_Y)) * N / n_X / n_Y
                                    if exitFlag:
                                        break
                                if exitFlag:
                                    continue
                                if J > J_m:
                                    J_m = J
                                    pathBuffer[r] = [k1, k2, k3]
        # renew robots' location
        loop = 20
        for i in range(loop):
            for r in range(n_R):
                k = pathBuffer[r][time % 3]
                robot[r][2] = robot[r][2] + k * steel / loop
                robot[r][0] = robot[r][0] + velocity / loop * math.cos(robot[r][2] - 0.5 * k * steel / loop)
                robot[r][1] = robot[r][1] + velocity / loop * math.sin(robot[r][2] - 0.5 * k * steel / loop)
                robotPath[r].append([robot[r][0], robot[r][1]])
            msg = np.array([robot[0][0] / 10, robot[0][1] / 10], dtype=np.float32)
            pub.publish(msg)
        if time % 150 == 149:
            diffN += 1
        time = time + 1
        rate.sleep()
    rospy.spin()
		
