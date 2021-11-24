#!/usr/bin/env python3
import numpy as np
from math import pi


from formation_common.utils import gen_line, gen_arc


PI = pi
V = 1.5         # m/s
MIN_R = 0.8/np.tan(20/180*PI)           # 2.2米左右
TARGET_SPEED = 3

ds_formation_transition = 5    # 队形调整间距
# 全局任务信息，初始化为1
task = 0
# n_car=1
# n_car = 2
# car_ids = [7]

# n_car = 3
# car_ids = [1,2,5]

n_car = 5
colors = [0,1,2]            # 0 pix, 1 极创, 2军交
car_ids= [i+1 for i in range(n_car)]
pix_ids = [i+1 for i in range(5)]

d_car = 2.5
isPlot = 0

# 前三个为障碍物锥桶，后三个为后方的敌方智能体
obs = np.array([[ -7.49,	16.28],
    [-10.76,	16.55],
    [-11.86,	13.48],
    [-7.98,17.21],
    [-11.36,17.38],
    [-12.51,14.23]])

# 演示场景中，我方静止队形时的位置
battle_pos=np.array([
    [-3.26,11.59],
    [-5.77,10.53],
    [-7.87,8.98],
])
battle_theta = -157.1848
battle_theta_norm = -90 + battle_theta

# 场地边界
boundary = np.array([[12.06,19.49],
    [4.51,19.77],
    [-22.23,19.89],
    [-22.68,-0.44],
    [-15.10,-18.49],
    [-6.83,-22.89],
    [11.80,-22.88],
    [13.49,-10.47],
    [13.70,14.47],
    [12.20,19.50]])


# 集结队形参数调整
r_car = 4           # pix建议车辆转弯半径
R = 12          # 某距离


