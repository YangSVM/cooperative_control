#!/usr/bin/env python3
from operator import pos
import numpy as np
from math import pi

from numpy.linalg.linalg import norm
from formation_common.utils import  gen_line, gen_arc
from formation_common.formation_zoo import FormationType, formation_line, formation_point
import matplotlib.pyplot as plt
from formation_common.config_formation_continous import *
import dubins
import math
from formation_common.formation_core import Assign

step_size = 1

''' 轨迹注意事项：
前面留长；掉头后给一小段直线
'''

def load_critial_points():
    ''' 任务 间 关键点。
    '''
    # pix小车位置
    critical_poses = np.array([
        [8, 0, -pi/2],                  # pix小车初始右侧起点
        [0,-14,pi/2],                   # 南侧集结点
        [0,14,-pi/2],               # 北侧集合点

    ])

    # 极创

    # 运输车
    return critical_poses


def load_formation_config_base(car_colors):
    ''' 默认5辆车不参加队形变换的配置
    '''
    n_group = len(car_colors)
    formation_begin_ls =[[0] for i in range(n_group)]
    formation_type_ls =[[FormationType.Line] for i in range(n_group)]
    ds_trans_ls = [[0] for i in range(n_group)]
    return formation_begin_ls, formation_type_ls, ds_trans_ls


def load_formation_turnning_config(car_colors, s_max):
    ''' 多辆车联合拐弯
    '''
    n_group = len(car_colors)
    formation_begin_ls =[[0, s_max- pi*r_car] for i in range(n_group)]
    formation_type_ls =[[FormationType.Line, FormationType.Line] for i in range(n_group)]
    ds_trans_ls = [[0, -1] for i in range(n_group)]
    
    return formation_begin_ls, formation_type_ls, ds_trans_ls


def load_task_base(poses=[]):
    ''' 通过控制点生成单车轨迹。 dubins
    Params:
        poses: [n_routes, n_stage, 3]. 3: x, y, yaw。 n_routes: 全局轨迹数量多少。所有车辆最终可能的轨迹。
    Return:
        routes: List. n_routes*n_points*2. 
    '''
    r = r_car

    n_routes = len(poses)
    n_stages = [len(poses[i])  for i in range(n_routes)]

    routes = []
    for i in range(n_routes):
        trajs = []
        for j in range(n_stages[i] -1):
            config = dubins.shortest_path(poses[i][j, :], poses[i][j+1, :], r)
            traj, s_list = config.sample_many(step_size)
            trajs.extend(traj)

        trajs = np.array(trajs)
        trajs = trajs[:, :2]

        # 最后补一段一米的直线
        theta = poses[i][-1, 2]
        traj = gen_line(poses[i][-1, :2], poses[i][-1, :2]+np.array([math.cos(theta), math.sin(theta)]),  ds=0.1)
        trajs = np.concatenate((trajs, traj), axis=0)

        routes.append(trajs)

    if n_routes > 1:
        car_colors = [[0,0],[0],[0,0]]      # 侦查任务。多列侦查
    elif n_routes == 1:
        car_colors = [[0 for i in range(n_car)]]

    # formation_begin_ls, formation_type_ls, ds_trans_ls = load_formation_config_base(car_colors)

    formation_begin_ls, formation_type_ls, ds_trans_ls = load_formation_turnning_config(car_colors, s_max=s_list[-1])

    return car_colors ,routes, formation_begin_ls, formation_type_ls, ds_trans_ls


# task: 集结(部署)
def build_up():
    poses = load_critial_points()
    poses_build_up  = poses[:2, :]              # 北侧往南
    poses_build_up = [poses_build_up]
    car_colors ,routes, formation_begin_ls, formation_type_ls, ds_trans_ls = load_task_base(poses_build_up)
    formation_type_ls[0][0] = FormationType.Vertical
    formation_begin_ls[0][1] -= 5
    return car_colors ,routes, formation_begin_ls, formation_type_ls, ds_trans_ls


# task: 侦查。2辆车左半圆掉头，2车右半圆掉头，1车直线掉头
def search(pos_start_center=[], pos_end_center=[]):
    '''侦查！！！
    使用 dubins car设计侦查路线
    '''
    critial_points = load_critial_points()

    routes = []
    n_routes = min(3, n_car)        # 侦查轨迹数目

    car_colors = [[] for i in range(n_routes)]          # 车辆分配。平均分，多的从左往右分
    n_mod = n_car%n_routes
    for i in range(n_routes):
        car_colors[i].append(0)
        if i<n_mod:
            car_colors[i].append(0)

    # 计算车辆的中心
    n_group_cars = [len(group_colors) for group_colors in car_colors]       # 每条轨迹分配了多少车
    d_car_ratio= {5:[2,1.5], 4:[1.5,1], 3:[1,1], 2:[1,0]}
    if len(pos_start_center) ==0:
        pos_start_center = critial_points[1, :]
        pos_starts = np.ones([3 ,3])* pos_start_center[ -1]      # n_car*3. x,y,yaw
        pos_starts[1, :] = pos_start_center

        pos_starts[0, :2] = formation_line(pos_start_center[:2], pos_start_center[2], n_car=3, d_car=d_car*d_car_ratio[n_car][0])[0, :]
        pos_starts[2, :2] = formation_line(pos_start_center[:2], pos_start_center[2], n_car=3, d_car=d_car*d_car_ratio[n_car][1])[2, :]
        
    if len(pos_end_center) ==0:
        pos_end_center = critial_points[2, :]
        pos_ends = np.ones([3 ,3])* pos_end_center[ -1]

        pos_ends[1, :] = pos_end_center        
        pos_ends[0, :2] = formation_line(pos_end_center[:2], pos_end_center[2], n_car=3, d_car=d_car*d_car_ratio[n_car][0])[0, :]
        pos_ends[2, :2] = formation_line(pos_end_center[:2], pos_end_center[2], n_car=3, d_car=d_car*d_car_ratio[n_car][1])[2, :]
    
    # 匈牙利分配
    pos_ends_xy, _=Assign(pos_starts, pos_ends)
    pos_ends[:, :2] = pos_ends_xy

    # 分三组路线前进。左边，中间，右边
    ds = 8
    pos_mids = np.array([
        [-ds-5, 0, pi/2],
        [0, 0, pi/2],
        [ds, 0, pi/2]
    ])

    step_size = 0.5
    r = r_car       # 转弯半径

    for i in range(n_routes):
        # 左中右三条路径. 如果只有3辆以下的车，后面的routes舍弃
        ci1 = dubins.shortest_path(pos_starts[i, :], pos_mids[i, :], r)
        traj, s1 = ci1.sample_many(step_size)
        ci2 = dubins.shortest_path(pos_mids[i, :], pos_ends[i, :] ,r)
        traj2, s2 = ci2.sample_many(step_size)
        traj.extend(traj2)
        traj = np.array(traj)
        routes.append(traj[:, :2])
        # plt.plot(traj[:,0], traj[:, 1], 'g*-')

    # plt.axis('equal') 
    
    # car_ids=[[1,2], [3],[4,5]]
    


    formation_begin_ls, formation_type_ls, ds_trans_ls = load_formation_turnning_config(car_colors, s1[-1]+s2[-1])
    return car_colors ,routes, formation_begin_ls, formation_type_ls, ds_trans_ls


# task: 侦查后返回。编队队形变换。
def after_search():
    r = r_car
    c1 = gen_line([0, R+5], [0,-R+r], ds=1)
    c2 = gen_arc([r, -R + r], -179.9,0,r, ds=0.1)
    c3 = gen_line([r*2, -R+2*r], [r*2, -R+2*r + 1], ds=1)       # 加一段小直线。方便掉头
    center_line = np.concatenate((c1, c2, c3), axis=0)
    formation_begin_s = [0,4, 14, 20]
    formation_types = [FormationType.Line, FormationType.Triangle, FormationType.Line, FormationType.Line]
    ds_trans = [ds_formation_transition for i in range(len(formation_begin_s))]
    ds_trans[-1] = -1

    car_colors = [[0 for i in range(n_car)]]
    return car_colors ,[center_line], [formation_begin_s], [formation_types], [ds_trans]

def battle(): 
    # # 打击：
    r = r_car
    end_pos = battle_pos[1,:]
    begin_pos = np.array([r*2, -R+2*r + 1])
    extend_vec = end_pos - begin_pos
    extend_vec = extend_vec/np.linalg.norm(extend_vec) * 5
    extend_pos  =begin_pos - extend_vec
    c1 = gen_line(extend_pos , begin_pos, ds =1)
    c2 = gen_line(begin_pos, battle_pos[1, :], ds=1)
    # center_line = np.concatenate((c1, c2), axis=0)
    center_line = np.array([extend_pos, end_pos]).reshape(2,2)
    formation_begin_s = [0, 2]
    formation_types = [FormationType.Line, FormationType.Line]
    ds_trans = [ds_formation_transition for i in range(len(formation_begin_s))]
    
    car_colors = [[0 for i in range(n_car)]]
    # plt.plot(center_line[:, 0], center_line[:, 1], 'b*-')
    # plt.axis('equal')
    return car_colors, [center_line], [formation_begin_s], [formation_types], [ds_trans]

# 二阶段打击：


# 占领：
def occupy():
    '''用直线队列方法通过障碍物并且占领
    '''
    global battle_pos, battle_theta, battle_theta_norm, obs
    pos_formations = []

    # 上一阶段的结束队形
    pos_start_center = np.copy(battle_pos[1, :])
    isAttackLeft = 1
    d_theta = 20
    d_pos_x = d_car/2
    if isAttackLeft==1:
        print('attack left')
        d_pos_x = -d_pos_x
    else:
        print('attack right')
        d_theta  = - d_theta
    pos_start = formation_line(pos_start_center+ [-d_pos_x, 0], battle_theta-d_theta, n_car, d_car)
    pos_formations.append(pos_start)

    # 围绕障碍物的队形
    # 单列通过。每列中心是障碍物中心
    theta_norm = (battle_theta_norm)/180*pi
    d_norm =  np.array([math.cos(theta_norm), math.sin(theta_norm)])
    pos_through_mid = (obs[0, :] + obs[1, :])/2 - d_norm *2.5
    pos_through  = formation_point(pos_through_mid, n_car)
    
    pos_formations.append(pos_through)

    # 北尽头拐弯处

    point_north_turn = pos_through_mid+d_norm*3
    pos_north_turn = formation_point(point_north_turn, n_car)
    pos_formations.append(pos_north_turn)

    # 返回集结点的队形
    poses = load_critial_points()
    end_pos = poses[2, :2].copy()
    # control point. 控制点.帮助拐弯
    end_pos_cp = end_pos.copy()
    end_pos_cp[1] += 0.5
    end_pos_cp[0] -= 1
    pos_line_cp =formation_line(end_pos_cp, 0, n_car, d_car)
    pos_formations.append(pos_line_cp)

    pos_line =formation_line(end_pos, 0, n_car, d_car)
    pos_formations.append(pos_line)
    pos_formations = np.array(pos_formations)

    theta_degs = np.array([battle_theta_norm,battle_theta_norm, battle_theta_norm, -90, -90])

    critical_state_selector = [0,1,2,3]
    car_colors = [[0 for i in range(n_car)]]

    # 生成一段小直线
    theta_line = (battle_theta + 90)/180*pi
    tiny_line = gen_line( pos_start_center+2*np.array([math.cos(theta_line), math.sin(theta_line)]), pos_start_center, ds=0.2)

    end_tiny_line  = gen_line(end_pos, end_pos+np.array([0, -2]), ds=0.2)
    routes = np.concatenate((pos_start_center, pos_through_mid, point_north_turn, end_pos_cp, end_pos), axis=0).reshape([-1,2])    
    routes = np.concatenate((tiny_line, routes, end_tiny_line), axis=0)

    # 
    # routes = [pos_start_center, pos_through_mid, point_north_turn, end_pos_cp, end_pos]
    formation_types = [FormationType.Line, FormationType.Point, FormationType.Point, FormationType.Point, FormationType.Line]
    formation_begin_s = [0,2,8,10,10]
    ds_trans = [0,5,0,0, -1]
    return car_colors, [routes], [formation_begin_s], [formation_types], [ds_trans]
    

def load_task_configs():
    ''' 把整个流程的轨迹完全记录
    '''
    car_color_list, routes_list, formation_begin_ls, formation_type_ls, ds_trans_ls, theta_degs_list = [], [], [], [], [], []
    n_task = 5
    for i in range(task, n_task):
        if i ==0:
            car_colors ,routes, formation_begin_s, formation_types, ds_trans = build_up()
        elif i==1:
            car_colors ,routes, formation_begin_s, formation_types, ds_trans = search()
        elif i==2:
            car_colors ,routes, formation_begin_s, formation_types, ds_trans = after_search()

        elif i==3:
            car_colors ,routes, formation_begin_s, formation_types, ds_trans = battle()

        elif i==4:
            car_colors, routes, formation_begin_s, formation_types, ds_trans  = occupy()

        car_color_list.append(car_colors)
        routes_list.append(routes)
        formation_begin_ls.append(formation_begin_s)
        formation_type_ls.append(formation_types)
        ds_trans_ls.append(ds_trans)
    
    return car_color_list, routes_list, formation_begin_ls, formation_type_ls, ds_trans_ls

if __name__=='__main__':
    search()