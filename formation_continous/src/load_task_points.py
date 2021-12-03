#!/usr/bin/env python3
from matplotlib import rcParams
import numpy as np
from math import  pi

from numpy.linalg.linalg import norm
from formation_common import global_planning
from formation_common import formation_zoo
from formation_common.utils import  gen_line, gen_arc
from formation_common.formation_zoo import FormationType, formation_line, formation_point, gen_formation
import matplotlib.pyplot as plt
from formation_common.config_formation_continous import *
from formation_common.global_planning import dubins_pf_pro
import dubins
import math
from formation_common.formation_core import Assign
from formation_stage_control import FormationStage
from formation_common.draw_lqr import draw_car
from formation_common.cubic_spline_planner import spline_expand
import copy
from math import radians as rad

step_size = 1

''' 轨迹注意事项：
前面留长；掉头后给一小段直线
'''

# 上一阶段的结束位置
last_end_poses = []




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
    if isinstance(s_max, list) and n_group == len(s_max):
        formation_begin_ls =[[0, s_max[i]- pi*r_pix] for i in range(n_group)]
    else:
        formation_begin_ls =[[0, max(0, s_max- pi*r_pix-5)] for i in range(n_group)]

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
    r = r_pix

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
        car_colors = [[0 for i in range(n_pix)]]

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
    pix_poses, jc_poses, tp_poses = load_critial_poses(i_pix=[0,1], i_jc=[0,1], i_tp=[0,1])
    formation_types  = [FormationType.Vertical, FormationType.Line]
    poses_build_up  = pix_poses[:2, ...]           # 右侧往南
    n_stage = poses_build_up.shape[0]     # 总共 队形阶段

    matches = np.arange(n_stage*n_pix).reshape([n_stage, n_pix])

    # 生成起止点的对应队形
    pos_stage = np.ones([2, n_pix, 3])          # 2stage, 3:xy yaw
    for i in range(2):
        pos_stage[i, :, :2] = gen_formation(poses_build_up[i, :2], poses_build_up[i, 2], n_pix, d_car, formation_types[i])
        pos_stage[i, :, 2] =poses_build_up[i, 2]
    _, matches_i = Assign(pos_stage[1,:, :2], pos_stage[0,:, :2])
    # 匹配结果取反。因为匈牙利不考虑朝向。匈牙利完全不正确...手动编写
    matches[i, :] = np.arange(n_pix)
    pos_stage[1, ...] = pos_stage[1, matches[i, :], :]

    # 然后生成各自轨迹
    ctrl_points_pix =np.array([
        [6,-6, -pi/2],
        [2, -10, pi], 
    ])
    individuals = []
    for j in range(n_pix):
        individual = dubins_pf_pro(pos_stage[:, j, :], ctrl_points_pix[0, :], step_size=0.1)
        individuals.append(individual)
    
    # plot_scenario(poses=pos_stage,individuals=individuals)

    # 运输车
    ctrl_points = tp_poses[0, :] + np.array([0, -5, 0])
    trajs = dubins_pf_pro(tp_poses, ctrl_points, r_tp)
    individual_tp = np.array(trajs)
    matches_tp = np.arange(1*len(color_ids[2])).reshape([1, len(color_ids[2])])

    # plt.plot(trajs[:, 0], trajs[:, 1], 'k-')

    # 极创
    # ctrl_point =  np.array([6, -5, -pi/2])
    ctrl_points_tp = np.ones([3,3])
    ctrl_points_tp[:2, :] = ctrl_points_pix
    ctrl_points_tp[2, :] = np.array([4, -5, +pi/2]) +tp_poses[-1, :]
    trajs = dubins_pf_pro(jc_poses, ctrl_points_pix, r_jc)
    individual_jc = np.array(trajs)
    matches_jc = np.arange(1*len(color_ids[2])).reshape([1, len(color_ids[2])])

    # plt.plot(trajs[:, 0], trajs[:, 1], 'k-')


    # 编队参数
    car_colors = [[0 for i in range(n_pix)], [1], [2]]
    n_group = len(car_colors)

    formation_begin_ls =[[0, 0.1], [0]]
    formation_type_ls =[[FormationType.Vertical, FormationType.Line] , [FormationType.Line]]
    ds_trans_ls = [[0, -1], [0]]

    fs = FormationStage.from_individual(individuals, formation_begin_ls[0], formation_type_ls[0], ds_trans_ls[0], matches=matches)
    fs_jc =  FormationStage.from_individual([individual_jc], formation_begin_ls[1], formation_type_ls[1], ds_trans_ls[1], matches=matches_jc)
    fs_tp =  FormationStage.from_individual([individual_tp], formation_begin_ls[1], formation_type_ls[1], ds_trans_ls[1], matches=matches_tp)

    fs.ddelta_s = [-2*i for i in range(n_pix)]
    s0 = fs.ddelta_s[n_pix//2]
    fs.ddelta_s = [s-s0 for s in fs.ddelta_s]           # 归一化 # 前车到后车，增加加速，减少减速 
    fs.ddelta_s[-1] -= 5            # 最后的车一开始轨迹较长，需要减一个量               

    fs_jc.v = 0.6*TARGET_SPEED

    return car_colors , [fs, fs_jc,fs_tp]


# task: 侦查。2辆车左半圆掉头，2车右半圆掉头，1车直线掉头
def search(pos_start_center=[], pos_end_center=[]):
    '''侦查！！！
    使用 dubins car设计侦查路线
    '''
    pix_points, jc_poses, tp_pos = load_critial_poses()

    routes = []
    n_routes = min(3, n_pix)        # 侦查轨迹数目

    car_colors = [[] for i in range(n_routes)]          # 车辆分配。平均分，多的从左往右分
    n_mod = n_pix%n_routes
    for i in range(n_routes):
        car_colors[i].append(0)
        if i < n_mod:
            car_colors[i].append(0)

    # 计算车辆的中心
    n_group_cars = [len(group_colors) for group_colors in car_colors]       # 每条轨迹分配了多少车
    d_car_ratio= {5:[1.5, 2], 4:[1,1.5], 3:[1,1], 2:[0,1]}
    delta_x ={5: 1}
    if len(pos_start_center) ==0:
        pos_start_center = pix_points[1, :]
        pos_starts = np.ones([3 ,3])* pos_start_center[ -1]      # n_car*3. x,y,yaw
        pos_starts[1, :] = pos_start_center

        pos_starts[0, :2] = formation_line(pos_start_center[:2], pos_start_center[2], n_car=3, d_car=d_car*d_car_ratio[n_pix][1])[0, :]
        pos_starts[2, :2] = formation_line(pos_start_center[:2], pos_start_center[2], n_car=3, d_car=d_car*d_car_ratio[n_pix][0])[2, :]
        
        # 进行整体偏移
        pos_starts[:, 0] = pos_starts[:,0] + delta_x[n_pix]


    if len(pos_end_center) ==0:
        pos_end_center = pix_points[2, :]
        pos_ends = np.ones([3 ,3])* pos_end_center[ -1]

        pos_ends[1, :] = pos_end_center        
        pos_ends[0, :2] = formation_line(pos_end_center[:2], pos_end_center[2], n_car=3, d_car=d_car*d_car_ratio[n_pix][0])[0, :]
        pos_ends[2, :2] = formation_line(pos_end_center[:2], pos_end_center[2], n_car=3, d_car=d_car*d_car_ratio[n_pix][1])[2, :]
        # 进行整体偏移
        pos_ends[:, 0] = pos_ends[:,0] + delta_x[n_pix]
    
    # 匈牙利分配
    pos_ends_xy, _=Assign(pos_starts, pos_ends)
    pos_ends[:, :2] = pos_ends_xy

    # 分三组路线前进。左边，中间，右边
    ds = 8
    pos_mids = np.array([
        [-ds-5, 5, pi/2],
        [-5, 5, pi/2],
        [6, 5, pi/2]
    ])

    step_size = 0.5
    r = r_pix       # 转弯半径

    s_max = []
    for i in range(n_routes):

        traj = dubins_pf_pro(np.array([pos_starts[i, :], pos_ends[i, :]]), ctrl_points=pos_mids[i, :], r=r_pix)
        routes.append(traj[:, :2])

        ci1 = dubins.shortest_path(pos_starts[i, :], pos_ends[i, :], r)
        traj, s1 = ci1.sample_many(step_size)

        s_max.append(s1[-1])


    n_group = len(car_colors)
    formation_begin_ls, formation_type_ls, ds_trans_ls = load_formation_turnning_config(car_colors, s_max)
    fs_list = [FormationStage.from_global(routes[i], len(car_colors[i]),\
            formation_begin_ls[i], formation_type_ls[i], ds_trans=ds_trans_ls[i]) for i in range(n_group)]
    
    i_car = 0
    d_turn = 1
    # for i in range(n_group):
    #     n_group_car = n_group_cars[i]
    #     for j in range(n_group_car):
    #         ddelta_s = -(n_pix - i_car)*d_turn + d_turn*(n_pix//2)
    #         fs_list[i].ddelta_s.append(ddelta_s)
    #         i_car += 1
    
    fs_list_delta_s = [-2, 0,1,2,3]
    i_car = 0
    for i in range(n_group):
        n_group_car = n_group_cars[i]
        for j in range(n_group_car):
            ddelta_s = fs_list_delta_s[i_car]
            fs_list[i].ddelta_s.append(ddelta_s)
            i_car += 1


    for i in range(3):
        plot_scenario(individuals=routes)
        plot_scenario(csps=fs_list[i].individual_csps)

    return car_colors, fs_list


# task: 侦查后返回。编队队形变换。
def after_search():
    pix_points, jc_poses, tp_pos = load_critial_poses()
    pos_start = pix_points[2, :]
    pos_end = pix_points[3, :]
    r = r_pix

    conf = dubins.shortest_path(pos_start, pos_end, r)
    traj, s_traj = conf.sample_many(step_size)
    tiny = gen_line(pos_end, pos_end + np.array([0, 1, 0]), ds=0.1, dim=3)       # 加一段小直线。方便掉头
    traj.extend(tiny)
    center_line = np.array(traj)[:, :2]

    # 小场地变换队形困难
    # formation_begin_s = [0,4, 14, 25]
    # formation_types = [FormationType.Line, FormationType.Triangle, FormationType.Line, FormationType.Line]
    # ds_trans = [ds_formation_transition for i in range(len(formation_begin_s))]
    # ds_trans[-1] = -1

    # 不变换队形
    car_colors = [[i for i in range(n_pix)]]
    formation_begin_ls, formation_type_ls, ds_trans_ls = load_formation_turnning_config(car_colors, s_traj[-1])
    # formation_begin_s = [0,0.1]
    # formation_types = [FormationType.Line, FormationType.Line]
    # ds_trans = [ds_formation_transition for i in range(len(formation_begin_s))]
    # ds_trans[-1] = -1
    
    fs = FormationStage.from_global(center_line, n_pix,formation_begin_ls[0], formation_type_ls[0], ds_trans=ds_trans_ls[0])
    fs.ddelta_s = [i*2 - n_pix for i in range(n_pix)]
    car_colors = [[0 for i in range(n_pix)]]
    plot_scenario(csps=fs.individual_csps)
    return car_colors ,[fs]

def battle(): 
    # 打击：
    pix_points, jc_poses, tp_pos = load_critial_poses()
    task_pos = pix_points[3:5, :]
    jc_poses = jc_poses[1:3, :]

    ctrl_point = (task_pos[0, :] + task_pos[1, :])/2           # 走到中间位置队形进行变换
    ctrl_point[2] = task_pos[1, 2] - 0.001
    center_line = dubins_pf_pro(task_pos, ctrl_point)[:, :2]


    formation_begin_s = [0, 5, 10]
    formation_types = [FormationType.Line, FormationType.Triangle, FormationType.Line]
    ds_trans = [ds_formation_transition for i in range(len(formation_begin_s))]
    ds_trans[1] = 8
    ds_trans[-1] = -1
    
    # 极创轨迹
    individual_jc = dubins_pf_pro(jc_poses, ctrl_points=ctrl_point+np.array([0,-5,0]), r=r_jc, len_end_line=0)
    matches_jc = np.arange(1*len(color_ids[2])).reshape([1, len(color_ids[2])])

    car_colors = [[0 for i in range(n_pix)],[1]]

    fs = FormationStage.from_global(center_line, n_pix, formation_begin_s, formation_types, d_car, ds_trans)
    fs_jc =  FormationStage.from_individual([individual_jc], [0], [FormationType.Line], [0], matches=matches_jc)
    fs_jc.v = 0.6*TARGET_SPEED
    plot_scenario(csps=fs_jc.individual_csps)
    return car_colors, [fs, fs_jc]

# 二阶段打击：停在给定地点进行测试
def battle_part1():
    # 打击阶段1：直线走到中间位置
    pix_points, jc_poses, tp_pos = load_critial_poses(i_pix=[3,4], i_jc=[1,2])
    mid_point = (pix_points[0, :] + pix_points[1, :])/2           # 走到中间位置队形进行变换
    mid_point[2] = pix_points[1, 2]

    pix_points[-1, :] = mid_point
    car_colors = [[0 for i in range(n_pix)], [1]]

    pix_route = dubins_pf_pro(pix_points)
    formation_begin_s = [0, 0.1]
    formation_types = [FormationType.Line,  FormationType.Line]
    ds_trans = [0, -1]
    fs = FormationStage.from_global(pix_route, n_pix, formation_begin_s, formation_types, d_car, ds_trans)
    
    jc_poses[1, :2] = (jc_poses[0, :2] + jc_poses[1, :2])/2     # 结束位置改为中点
    # jc_poses[1, 2] =  jc_poses[1, 2]
    pix_route = dubins_pf_pro(jc_poses)
    fs_jc = FormationStage.from_global(pix_route, 1, formation_begin_s, formation_types, d_car, ds_trans)
    
    plot_scenario(csps=fs_jc.individual_csps)
    plot_scenario(csps=fs.individual_csps)
    return car_colors, [fs, fs_jc]

def battle_part2():
    # 打击阶段2：三角队形走到打击位置
    pix_points, jc_poses, tp_pos = load_critial_poses(i_pix=[3,4], i_jc=[1,2])
    mid_point = (pix_points[0, :] + pix_points[1, :])/2           # 走到中间位置队形进行变换
    mid_point[2] = pix_points[1, 2]

    pix_points[0, :] = mid_point
    car_colors = [[0 for i in range(n_pix)], [1]]

    pix_route = dubins_pf_pro(pix_points)
    formation_begin_s = [0, 0.1]
    formation_types = [FormationType.Line,  FormationType.Triangle]
    ds_trans = [0, -1]
    fs = FormationStage.from_global(pix_route, n_pix, formation_begin_s, formation_types, d_car, ds_trans)

    jc_poses[0, :2] = (jc_poses[0, :2] + jc_poses[1, :2])/2             # 起始位置改成中点
    jc_poses[0, 2] =  jc_poses[1, 2]

    pix_route = dubins_pf_pro(jc_poses)
    fs_jc = FormationStage.from_global(pix_route, 1, formation_begin_s, formation_types, d_car, ds_trans)
    
    plot_scenario(csps=fs_jc.individual_csps)
    plot_scenario(csps=fs.individual_csps)

    global last_end_poses
    last_end_poses = []
    for i in range(n_pix):
        s =  fs.individual_csps[i].s[-1]- 0.01
        tmp = fs.individual_csps[i].calc_pose(s)
        last_end_poses.append(tmp)

    return car_colors, [fs, fs_jc]


# 围捕
def pursuit():
    ''' 三角队形直接围捕对方车辆
    '''
    pix_points,_,_ = load_critial_poses(i_pix=[4])
    # 队形特殊性...现在是直接给三个位置
    global last_end_poses, battle_theta_norm
    begin_pose = copy.deepcopy(last_end_poses)
    begin_pose = np.array(begin_pose)
    # end_pos = np.array(copy.deepcopy(begin_pose))         # 其他车不动
    end_pos = np.ones([n_pix, 3])
    end_pos[:, :2] = formation_line(pix_points[0, :], battle_theta_norm/180*pi, n_pix, d_car)        # 其他车稍微动
    r_cap = [1.5, 1.5, 4.5]
    thetas = [10, 190, -60]
    end_pos[1:4, :2] = formation_zoo.formation_capture(obs[4, :],  3,  r_cap, thetas=[x/180*pi for x in thetas])
    end_pos[:, 2] = battle_theta_norm*pi/180
    last_end_poses = end_pos
    # plot_scenario(poses=begin_pose)
    # plot_scenario(poses=end_pos, style='r*')

    matches = np.arange(2*n_pix).reshape([2, n_pix])
    _, matches_i = Assign(np.array(begin_pose), end_pos)
    
    end_pos = end_pos[matches_i, :]
    matches[1, :] = matches_i
    
    individuals = []
    for i in range(n_pix):
        individual = dubins_pf_pro(np.array([begin_pose[i, :], end_pos[i, :]]), len_end_line=1e-3)
        individuals.append(individual)
    
    fs = FormationStage.from_individual(individuals, [0, 0.1], [FormationType.Arbitrary, FormationType.Arbitrary], ds_trans=[0, -1], matches=matches)
    plot_scenario(individuals=individuals)
    car_colors = [[0 for i in range(n_pix)]]
    return car_colors, [fs]


# 占领：手描路径点-配合单阶段打击且不围捕
def occupy():
    '''用直线队列方法通过障碍物并且占领
    '''
    global battle_pos, battle_theta, battle_theta_norm, obs
    pos_formations = []

    # 上一阶段的结束队形
    pos_start_center = np.copy(battle_pos[1, :])

    pos_start = formation_line(pos_start_center, battle_theta_norm, n_pix, d_car)
    pos_formations.append(pos_start)

    # 围绕障碍物的队形
    # 单列通过。每列中心是障碍物中心
    theta_norm = (battle_theta_norm)/180*pi
    d_norm =  np.array([math.cos(theta_norm), math.sin(theta_norm)])
    pos_through_mid = (obs[0, :] + obs[1, :])/2 - d_norm *2.5
    pos_through  = formation_point(pos_through_mid, n_pix)
    
    pos_formations.append(pos_through)

    # 北尽头拐弯处

    point_north_turn = pos_through_mid+d_norm*3
    pos_north_turn = formation_point(point_north_turn, n_pix)
    pos_formations.append(pos_north_turn)

    # 返回集结点的队形
    pix_poses, jc_poses, tp_pos = load_critial_poses()
    end_pos = pix_poses[2, :2].copy()
    # control point. 控制点.帮助拐弯
    end_pos_cp = end_pos.copy()
    end_pos_cp[1] += 0.5
    end_pos_cp[0] -= 1
    pos_line_cp =formation_line(end_pos_cp, 0, n_pix, d_car)
    pos_formations.append(pos_line_cp)

    pos_line =formation_line(end_pos, 0, n_pix, d_car)
    pos_formations.append(pos_line)
    pos_formations = np.array(pos_formations)

    theta_degs = np.array([battle_theta_norm,battle_theta_norm, battle_theta_norm, -90, -90])

    critical_state_selector = [0,1,2,3]
    car_colors = [[0 for i in range(n_pix)]]

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
    fs_list = [FormationStage.from_global(route, n_pix, \
            formation_begin_s, formation_types, ds_trans=ds_trans) for i in range(n_group)]
    return car_colors, fs_list
    
def occupy2():
    pix_begin = last_end_poses
    pix_points, jc_points, tp_points = load_critial_poses(i_pix=[4, 5], i_jc=[-2, -1], i_tp=[-2, -1])

    pix_stages = np.ones([2, n_pix, 3])

    pix_stages[0, ...] = copy.deepcopy(pix_begin)
    pix_end  = formation_line(pix_points[-1, :2], pix_points[-1, 2], n_pix, d_car)
    pix_stages[1, ..., :2] = pix_end
    pix_stages[1, :, 2] = pix_points[-1, 2]
    matches_i = np.array([4,1,3,2,0])       #手动设置.重新分配
    pix_stages[1, :, :] = pix_stages[1, matches_i, :]

    # 单列通过。每列中心是障碍物中心
    theta_norm = (battle_theta_norm)/180*pi
    d_norm =  np.array([math.cos(theta_norm), math.sin(theta_norm)])
    pos_through_mid = (obs[0, :] + obs[1, :])/2 - d_norm *2.5

    ctrl_points = np.array([
        [0,0,0],                # pos_through_mid。 原本障碍物中心
        [0,0,0],
        [-5, 20, 0],
        [-2, 20, 0],
        [-6, 18, rad(-20)],               # 4 右内侧控制点
        [-15, 18, 0],                           # 5 左外侧控制点
    ])
    ctrl_points[0, :2] = pos_through_mid
    ctrl_points[0, 2] = battle_theta_norm/180*pi

    point_north_turn = pos_through_mid+d_norm*3
    ctrl_points[1, :2] = point_north_turn
    ctrl_points[1, 2] = battle_theta_norm/180*pi

    # 每辆车使用第几个控制点s
    i_ctrl= {0:[5, 4], 1:[2,3], 2:[4], 3:[1, 2, 3], 4:[0,1,2,3]}
    individuals = []
    for i in range(n_pix):
        individual = dubins_pf_pro(pix_stages[:, i, :], ctrl_points=ctrl_points[i_ctrl[i]], len_end_line=1, r=1.5)
        individuals.append(individual)

    matches = np.arange(2*n_pix).reshape([2, n_pix])
    matches[1, :] = matches_i
    fs = FormationStage.from_individual(individuals, [0, 0.1], [FormationType.Arbitrary, FormationType.Arbitrary], [0, -1], matches)
    fs.ddelta_s = [-i*2 + n_pix for i in range(n_pix)]
    car_colors = [[0 for i in range(n_pix)], [1], [2]]
    plt.clf()
    plot_scenario(poses=ctrl_points, style='r*')
    plot_scenario(individuals=individuals)
    fs.priority = np.array([4,0,3,1,2])         # 每辆车的通行权(通行次序)。数字越小越先通行。
    fs.v = 2            # 此场景速度降低

    # jc
    individuals_jc = dubins_pf_pro(jc_points, r=r_jc)
    individuals_tp= dubins_pf_pro(tp_points, r=r_tp)
    fs_jc = FormationStage.from_individual([individuals_jc], [0], [FormationType.Line], [0], np.arange(1).reshape([1, 1]) )
    fs_tp = FormationStage.from_individual([individuals_tp], [0], [FormationType.Line], [0], np.arange(1).reshape([1, 1]) )

    # plot_scenario(individuals=[individuals_tp])
    return car_colors, [fs, fs_jc, fs_tp]
    # pass

def load_task_configs():
    ''' 把整个流程的轨迹完全记录
    '''
    # car_color_list, routes_list, formation_begin_ls, formation_type_ls, ds_trans_ls, theta_degs_list = [], [], [], [], [], []
    car_color_list, fs_lists = [], []

    n_task = 7
    for i in range(task, n_task):
        if i ==0:
            car_colors , fss = build_up()
        elif i==1:
            car_colors , fss = search()
        elif i==2:
            car_colors , fss = after_search()

        elif i==3:
            car_colors , fss = battle_part1()

        elif i==4:
            car_colors , fss = battle_part2()
        elif i==5:
            car_colors , fss = pursuit()
        elif i==6:
            car_colors , fss = occupy2()

        car_color_list.append(car_colors)
        fs_lists.append(fss)
        # routes_list.append(routes)
        # formation_begin_ls.append(formation_begin_s)
        # formation_type_ls.append(formation_types)
        # ds_trans_ls.append(ds_trans)
    
    return car_color_list,fs_lists


def plot_scenario(poses=[], individuals=[], csps=[], style='b*'):
    ''' 画出当前的预定轨迹，查看是否存在碰撞
    '''
    plt.figure(2)
    plt.plot(boundary[:,0], boundary[:,1], 'r-')
    plt.plot(restrict_boundary[:, 0], restrict_boundary[:, 1], 'r-*')

    plt.axis('equal')

    # 画出敌方车辆和锥桶
    for i_agent in range(obs.shape[0]):
        draw_car(obs[i_agent, 0], obs[i_agent, 1], (battle_theta+90)/180*pi, 0, 'b')

    if len(poses)>0:
        for i in range(len(poses)):
            plt.plot(poses[i][..., 0], poses[i][..., 1], style)

    line_color = ['r', 'g', 'b', 'y', 'k']
    if len(individuals)>0:
        for i in range(len(individuals)):
            plt.plot(individuals[i][:, 0], individuals[i][:, 1], line_color[i]+'-.')

    if len(csps)>0:
        for i in range(len(csps)):
            x,y,_,_,_ = spline_expand(csps[i])
            plt.plot(x, y, 'r-')

    plt.axis('equal')


if __name__=='__main__':
    # search()
    # build_up()
    # battle_part1()
    # battle_part2()
    load_task_configs()
    # pursuit()
