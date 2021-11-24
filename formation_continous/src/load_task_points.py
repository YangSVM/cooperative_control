#!/usr/bin/env python3
import numpy as np
from math import pi

from numpy.linalg.linalg import norm
from formation_common.utils import  gen_line, gen_arc
from formation_common.formation_zoo import FormationType, formation_line, formation_point, gen_formation
import matplotlib.pyplot as plt
from formation_common.config_formation_continous import *
import dubins
import math
from formation_common.formation_core import Assign
from formation_stage_control import FormationStage

step_size = 1

''' 轨迹注意事项：
前面留长；掉头后给一小段直线
'''

def load_critial_points():
    ''' 任务 间 关键点。
    '''
    # pix小车位置
    critical_poses = np.array([
        [8, 0, -pi/2],                  # pix小车初始右侧起点。集结开始                     0
        [0,-14,pi/2],                   # 南侧集结点。集结结束，侦查开始                 1
        [0,14,-pi/2],               # 北侧集合点。侦查结束，侦查返回开始。        2
        [0.1,-14,pi/2],                   # 南侧集结点。侦查返回结束。打击开始。 3
        [0, 0, 0],                              # 打击静止位置。打击结束。占领开始          4
        [0, -14, -pi/2],                      # 占领结束                                                            5
    ])
    global battle_pos, battle_theta_norm
    critical_poses[4, :2] = battle_pos[1, :]
    critical_poses[4, 2] = battle_theta_norm/180*pi


    # 极创

    # 运输车
    return critical_poses

def dubins_pf_pro(begin, end, ctrl_points, r=r_car, step_size=step_size):
    ''' dubin path finding.
    '''
    pass


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
    # n_group = len(car_colors)
    # fs_list = [FormationStage.from_global(routes[i], \
    #         formation_begin_ls[i], formation_type_ls[i], ds_trans=ds_trans_ls[i]) for i in range(n_group)]

    return car_colors, routes, formation_begin_ls, formation_type_ls, ds_trans_ls


# task: 集结(部署)
def build_up():
    ''' 各自规划轨迹，并配置队形参数
    '''
    poses_critical = load_critial_points()
    formation_types  = [FormationType.Vertical, FormationType.Point, FormationType.Line]
    ctrl_point= np.array([8, -5, -pi/2])
    poses_build_up  = np.array([
        poses_critical[0, :],
        ctrl_point,
        poses_critical[1, :] ])             # 右侧往南
    n_stage = poses_build_up.shape[0]     # 总共 队形阶段

    poses_stages = []

    # 生成对应位置
    matches = np.arange(n_stage*n_car).reshape([n_stage, n_car])
    for i_stage in range(n_stage):
        if i_stage == 0:
            d_car = 2.5
        poses = gen_formation(poses_build_up[i_stage, :2], poses_build_up[i_stage, 2], n_car, d_car, formation_types[i_stage])
        if i_stage>0:
            _, matches_i = Assign(poses_stages[i_stage-1][:, :2], poses) 
            if i_stage == n_stage - 1:

                matches_i = np.array([n_car-1-match for match in matches_i])
                poses = [poses[match] for match in matches_i]      # 匈牙利分配.分配结果不满意，手动修改
            else:
                poses = [poses[match] for match in matches_i] 
            
            # if i_stage !=1:      # 删掉控制点的
            matches_i= matches_i[matches[i_stage-1, :]]
            matches[i_stage, :] = matches_i
            poses = np.array(poses)

        poses_stages.append(poses)
    
    matches = matches[[0,2], :]         
    # 删掉控制点的
    # matches = np.zeros([n_stage-1, n_car])    
    # matches[1, :] = 


    # 然后生成各自轨迹
    individuals = []
    for j in range(n_car):
        traj_icar = []
        for i in range(n_stage-1):
            poses = np.ones([2,3])
            poses[0, :2] = poses_stages[i][j, :2]
            poses[1, :2] = poses_stages[i+1][j, :2]
            poses[:2, 2] = poses_build_up[i:i+2, 2]

            ci1 = dubins.shortest_path(poses[0, :], poses[1, :], r_car)
            traj, _ = ci1.sample_many(step_size)    
            traj_icar.extend(traj)

        # 加一小段直线
        end_pos = traj_icar[-1]
        traj = gen_line(end_pos[:2], end_pos[:2] +np.array( [0, 1]), dim=3)
        traj_icar.extend(traj)

        traj_icar = np.array(traj_icar)
        individuals.append(np.array(traj_icar))
    
    # style = ['b', 'g', 'r', 'c' ,'k']
    # for i, individual in enumerate(individuals):
    #     plt.plot(individual[:,0] + 5*i, individual[:,1],  style[i] +'*')        # plot时横向分开
    # plt.axis('equal')

    car_colors = [[0 for i in range(n_car)]]
    n_group = len(car_colors)
    poses_build_up = [poses_build_up]
    formation_begin_ls =[[0, 0.1] for i in range(n_group)]
    formation_type_ls =[[FormationType.Vertical, FormationType.Line] for i in range(n_group)]
    ds_trans_ls = [[0, -1] for i in range(n_group)]

    fs = FormationStage.from_individual(individuals, formation_begin_ls[0], formation_type_ls[0], ds_trans_ls[0], matches=matches)
    fs.ddelta_s = [-2*i for i in range(n_car)]
    s0 = fs.ddelta_s[n_car//2]
    fs.ddelta_s = [s-s0 for s in fs.ddelta_s]           # 归一化
    fs.ddelta_s[-1] -= 5                # 前车到后车，增加加速，减少减速 
    return car_colors , [fs]


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
        if i < n_mod:
            car_colors[i].append(0)

    # 计算车辆的中心
    n_group_cars = [len(group_colors) for group_colors in car_colors]       # 每条轨迹分配了多少车
    d_car_ratio= {5:[1.5, 2], 4:[1,1.5], 3:[1,1], 2:[0,1]}
    if len(pos_start_center) ==0:
        pos_start_center = critial_points[1, :]
        pos_starts = np.ones([3 ,3])* pos_start_center[ -1]      # n_car*3. x,y,yaw
        pos_starts[1, :] = pos_start_center

        pos_starts[0, :2] = formation_line(pos_start_center[:2], pos_start_center[2], n_car=3, d_car=d_car*d_car_ratio[n_car][1])[0, :]
        pos_starts[2, :2] = formation_line(pos_start_center[:2], pos_start_center[2], n_car=3, d_car=d_car*d_car_ratio[n_car][0])[2, :]
        
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
    

    n_group = len(car_colors)
    formation_begin_ls, formation_type_ls, ds_trans_ls = load_formation_turnning_config(car_colors, s1[-1]+s2[-1])
    fs_list = [FormationStage.from_global(routes[i], len(car_colors[i]),\
            formation_begin_ls[i], formation_type_ls[i], ds_trans=ds_trans_ls[i]) for i in range(n_group)]
    
    i_car = 0
    d_turn = 2
    for i in range(n_group):
        n_group_car = n_group_cars[i]
        for j in range(n_group_car):
            ddelta_s = -(n_car - i_car)*d_turn + d_turn*(n_car//2)
            fs_list[i].ddelta_s.append(ddelta_s)
            i_car += 1

    return car_colors, fs_list


# task: 侦查后返回。编队队形变换。
def after_search():
    critical_poses = load_critial_points()
    pos_start = critical_poses[2, :]
    pos_end = critical_poses[3, :]
    r = r_car
    # c1 = gen_line(pos_start, [0,-R+r], ds=1)
    # c2 = gen_arc(pos_end, -179.9,0,r, ds=0.1)
    # c3 = gen_line([r*2, -R+2*r], [r*2, -R+2*r + 1], ds=1)       # 加一段小直线。方便掉头

    # center_line = np.concatenate((c1, c2, c3), axis=0)

    conf = dubins.shortest_path(pos_start, pos_end, r)
    traj, _ = conf.sample_many(step_size)
    tiny = gen_line(pos_end, pos_end + np.array([0, 1, 0]), ds=0.1, dim=3)       # 加一段小直线。方便掉头
    traj.extend(tiny)
    center_line = np.array(traj)[:, :2]

    formation_begin_s = [0,4, 14, 25]
    formation_types = [FormationType.Line, FormationType.Triangle, FormationType.Line, FormationType.Line]
    ds_trans = [ds_formation_transition for i in range(len(formation_begin_s))]
    ds_trans[-1] = -1
    
    fs = FormationStage.from_global(center_line, n_car,formation_begin_s, formation_types, ds_trans=ds_trans)
    fs.ddelta_s = [-i*2 + n_car for i in range(n_car)]
    car_colors = [[0 for i in range(n_car)]]
    return car_colors ,[fs]

def battle(): 
    # 打击：
    pix_global_points = load_critial_points()
    task_pos = pix_global_points[3:5, :]

    ctrl_points = (task_pos[0, :] + task_pos[1, :])/2           # 走到中间位置队形进行变换
    ctrl_points[2] = task_pos[1, 2]
    route = np.concatenate((task_pos[0, :], ctrl_points, task_pos[1, :]), axis=0).reshape([3,-1])

    r = r_car

    trajs = []
    for i in range(route.shape[0]-1):
        conf = dubins.shortest_path(route[i, :], route[i+1, :], r)
        traj, _ = conf.sample_many(step_size)
        trajs.extend(traj)
    center_line = np.array(trajs)[:, :2]

    formation_begin_s = [0, 5, 10]
    formation_types = [FormationType.Line, FormationType.Triangle, FormationType.Line]
    ds_trans = [ds_formation_transition for i in range(len(formation_begin_s))]
    ds_trans[1] = 8
    ds_trans[-1] = -1
    
    car_colors = [[0 for i in range(n_car)]]

    fs = FormationStage.from_global(center_line, n_car, formation_begin_s, formation_types, d_car, ds_trans)
    # fs.ddelta_s = [-i*2 + n_car for i in range(n_car)]
    return car_colors, [fs]
    # return car_colors, [center_line], [formation_begin_s], [formation_types], [ds_trans]

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
    route = np.concatenate((pos_start_center, pos_through_mid, point_north_turn, end_pos_cp, end_pos), axis=0).reshape([-1,2])    
    route = np.concatenate((tiny_line, route, end_tiny_line), axis=0)

    # 
    # routes = [pos_start_center, pos_through_mid, point_north_turn, end_pos_cp, end_pos]
    formation_types = [FormationType.Line, FormationType.Point, FormationType.Point, FormationType.Point, FormationType.Line]
    formation_begin_s = [0,2,8,10,10]
    ds_trans = [0,5,0,0, -1]
    
    n_group = len(car_colors)
    fs_list = [FormationStage.from_global(route, n_car, \
            formation_begin_s, formation_types, ds_trans=ds_trans) for i in range(n_group)]
    return car_colors, fs_list
    

def load_task_configs():
    ''' 把整个流程的轨迹完全记录
    '''
    # car_color_list, routes_list, formation_begin_ls, formation_type_ls, ds_trans_ls, theta_degs_list = [], [], [], [], [], []
    car_color_list, fs_lists = [], []

    n_task = 5
    for i in range(task, n_task):
        if i ==0:
            car_colors , fss = build_up()
        elif i==1:
            car_colors , fss = search()
        elif i==2:
            car_colors , fss = after_search()

        elif i==3:
            car_colors , fss = battle()

        elif i==4:
            car_colors , fss = occupy()

        car_color_list.append(car_colors)
        fs_lists.append(fss)
        # routes_list.append(routes)
        # formation_begin_ls.append(formation_begin_s)
        # formation_type_ls.append(formation_types)
        # ds_trans_ls.append(ds_trans)
    
    return car_color_list,fs_lists

if __name__=='__main__':
    # search()
    build_up()