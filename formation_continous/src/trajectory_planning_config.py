#!/usr/bin/env python3

"""
单车轨迹规划参数

"""
from formation_common.config_formation_continous import TARGET_SPEED
# Parameter
MAX_SPEED = 50/3.6 # maximum speed [m/s]
MAX_ACCEL = 3.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1  # maximum curvature [1/m]

D_ROAD_W = 0.6  # road width sampling length [m]
ROAD_WIDTH_START = -1.2
ROAD_WIDTH_END = 1.2

DT = 0.2  # 轨迹点间隔
T_PLANNING = 3.0  # min prediction time [s]
# TARGET_SPEED = 1.5 # target speed [m/s]
D_T_S = TARGET_SPEED   # target speed sampling length [m/s]
# N_S_SAMPLE = 1  # sampling number of target speed
N_S_SAMPLE = 10  # sampling number of target speed

ROBOT_RADIUS = 1  # robot radius [m]
t_thre=1.5 #碰撞检测时间阈值[s]

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 10
K_LAT = 1.0
K_LON = 3.0

show_animation = False

