#!/usr/bin/env python3
import numpy as np
from math import pi
import math


PI = pi
V = 1.5         # m/s
MIN_R = 0.8/np.tan(20/180*PI)           # 2.2米左右
TARGET_SPEED = 2

ds_formation_transition = 5    # 队形调整间距

# 全局任务信息，初始化为0
task = 0


n_pix = 5
colors = [0,1,2]            # 0 pix, 1 极创, 2军交
color_ids={
    0: [i+1 for i in range(n_pix)],        # 0。pix
    1: [6],                                                 # 1。极创
    2: [7]                                                   #2。军交
}

car_ids= []
for v in color_ids.values():
    car_ids.extend(v) 

pix_ids = color_ids[0]
jc_ids = color_ids[1]
n_car = len(car_ids)

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
dbattle_pos = battle_pos[1, :] - battle_pos[0, :]
battle_theta = math.atan2(dbattle_pos[1], dbattle_pos[0])*180/pi        # -157度。车身左侧朝向
battle_theta_norm = -90 + battle_theta                                                              # 车头朝向

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


# 更小的场地边界(考虑车位)
restrict_boundary = np.array([
    [8, 19],
    [-22, 19],
    [-22, 0],
    [-18, -10],
    [8,-10],
    [8, 19]
])


# 集结队形参数调整
r_pix = MIN_R+0.4           # pix建议车辆转弯半径

r_tp = 3.99            # TransPort 运输车转弯半径(军交给的是4)
r_jc = 3.99


def load_critial_poses(i_pix=[], i_jc=[], i_tp=[]):
    ''' 任务 间 关键点。
    '''
    # pix小车位置
    # pix_poses = np.array([
    #     [6, 0, -pi/2],                  # pix小车初始右侧起点。集结开始                     0
    #     [0,-14,pi/2],                   # 南侧集结点。集结结束，侦查开始                 1
    #     [5,12,-pi/2],               # 北侧集合点。侦查结束，侦查返回开始。        2
    #     [0.1,-14,pi/2],                   # 南侧集结点。侦查返回结束。打击开始。 3
    #     [0, 0, 0],                              # 打击静止位置。打击结束。占领开始          4
    #     [5, -14, -pi/2],                      # 占领结束                                                            5
    # ])

    #  小场地展示专用
    pix_poses = np.array([
        [6, 0, -pi/2],                          # pix小车初始右侧起点。集结开始                        0
        [-3, -3  ,pi/2],                        # 南侧集结点。集结结束，侦查开始                    1
        [2+1, 12,-pi/2],                    # 北侧集合点。侦查结束，侦查返回开始。       2
        [0,0,0],                                    # 南侧集结点。侦查返回结束。打击开始。       3
        [0, -3, 5/180*pi],                 # 打击静止位置。打击结束。占领开始               4
        [0, 0, 0],                                   # 占领结束                                                                    5
    ])

    # 南侧的集结点默认处于相同位置
    pix_poses[3, :] = pix_poses[1, :] + pix_poses[3, :]
    # 占领地点和北侧集结点默认相同
    pix_poses[5, :] = pix_poses[5, :] + pix_poses[2, :]

    global battle_pos, battle_theta_norm
    # battle_theta_norm = 145
    pix_poses[4, :2] += battle_pos[1, :]
    battle_theta_norm_rad = battle_theta_norm/180*pi 
    pix_poses[4, 2] += battle_theta_norm_rad

    # 极创
    # 偏移量
    jc_poses = np.array([
        [0, 7, 0],                      # 右侧开始集结。集结开始。
        [0,-2,0],                       # 南侧集结点。集结结束。打击开始位置
        [0,0,0],                        # 打击点位置。打击结束，占领开始位置
        [0, -5, 0]                       # 占领结束位置
    ])
    jc_poses[2, :2] = -5*np.array([math.cos(battle_theta_norm_rad), math.sin(battle_theta_norm_rad)])       # 相对pix往后5米
    jc_poses = jc_poses + pix_poses[[0,1,4, 5], :]

    # 运输车: transport vehicle. 运输车最好并排停在左侧
    tp_poses = np.array([
        [-15, 0, -pi/2],           # 运输车初始左侧起点。集结开始                     0
        [-7.5, 0, 0],                   # 南侧集结点。集结结束，占领开始                 1
        [3.5, -7, 0],                   # 占领结束                                                                  2
    ])
    tp_poses[0,  1] = pix_poses[0,1] +7
    tp_poses[1, :] +=  pix_poses[1, :]
    tp_poses[2, :] +=  pix_poses[5, :]

    if not isinstance(i_pix, list) or len(i_pix)>0:
        pix_poses = pix_poses[i_pix, :]
    if not isinstance(i_jc, list) or len(i_jc)>0:
        jc_poses = jc_poses[i_jc, :]
    if not isinstance(i_tp, list) or len(i_tp)>0:
        tp_poses = tp_poses[i_tp, :]
    
    return pix_poses, jc_poses, tp_poses