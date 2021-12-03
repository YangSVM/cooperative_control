#!/usr/bin/env python3

'''地图文件。描述不同任务的全局路径规划。csp。
'''
from math import e
import copy
from operator import ipow
import time
from typing import List, Tuple


from formation_common.formation_zoo import FormationType
from formation_common.formation_zoo import *
from formation_common.config_formation_continous import *
from formation_common.cubic_spline_planner import Spline2D, cartesian2frenet, spline_expand
import matplotlib.pyplot as plt
from formation_common.formation_core import Assign, FormationState, judge_formation_stage
import dubins
from formation_common.utils import  gen_line, gen_arc


'''
输入一条中心轨迹。给定路径点，以及路径轨迹。
'''

def generate_route_with_formation(center_line: np.ndarray, formation_input: List, formation_types: List[FormationType],
n_car: int, d_car: float =2.5, input_type: int=0, ds_trans=[])-> Tuple[Spline2D, List[Spline2D], np.ndarray, np.ndarray]:
    '''
    
    Params:
        中心线轨迹（用于生成全局的样条曲线），队形起始点s坐标，队形参数，编队转换过程距离。
        input_type: 0时输入的是s坐标。1时输入的是x,y坐标。
        ds_trans: 队形转换距离。默认每个队形的转换距离都是 ds_formation_trans。
        theta_degs: 生成队形时的角度
    Returns:
        global_csp: Spline2D.
        所有车辆的参考轨迹
    
    '''
    global_csp = Spline2D(center_line[:,0], center_line[:,1])

    formation_input = np.array(formation_input)
    # 计算队形的起始位姿。以及辅助的s坐标
    n_stage = formation_input.shape[0]
    formation_begin_pos = np.zeros([n_stage, 3])       # x,y,yaw(弧度制)
    formation_begin_s = np.zeros([n_stage, 1])             # s坐标
    if input_type ==0:
        formation_begin_s = formation_input
        for i, s in enumerate(formation_input):
            if s>global_csp.s[-1]:
                print('error: global planning used ')
            formation_begin_pos[i, 0], formation_begin_pos[i, 1] = global_csp.calc_position(s)
            formation_begin_pos[i, 2] = global_csp.calc_yaw(s)
    elif input_type==1:
        formation_begin_pos[:, :2] = formation_input
        for i in range(formation_begin_pos.shape[0]):
            formation_begin_s[i] = cartesian2frenet(formation_begin_pos[i, 0], formation_begin_pos[i, 1], global_csp)
            formation_begin_pos[i, 2] = global_csp.calc_yaw(formation_begin_s[i])
    else:
        assert False, print('error input of params')
    
    # 循环创建队形 ndarray. [stage, n_car, 2].表示每个阶段的每辆车的xy坐标
    formation_poses = np.zeros([2*n_stage - 1, n_car, 2]) 

    ds_trans = ds_trans_init(ds_trans, n_stage, formation_begin_s, global_csp.s[-1]-1)

    for i in range(n_stage):
        theta = formation_begin_pos[i, 2]
        
        if i==0:
            formation_poses[i, ...] = gen_formation(formation_begin_pos[i, :2], theta, n_car, d_car, formation_types[i])
        else:
            # 先生成初始队形（上一个阶段的队形，这一个阶段的位置）
            formation_poses[2*i-1,...] = gen_formation(formation_begin_pos[i, :2], theta, n_car, d_car, formation_types[i-1])
            # 再生成结束队形（这个阶段的队形，较远处的位置）

            s_next = formation_begin_s[i] + ds_trans[i]
            s_next = min(s_next, global_csp.s[-1]-1)          # 超长的直接干掉
            ds_trans[i] = s_next - formation_begin_s[i]
            pos_next = global_csp.calc_position(s_next)
            yaw_next = global_csp.calc_yaw(s_next)
            formation_poses[2*i,...] = gen_formation(pos_next, yaw_next, n_car, d_car, formation_types[i])

    # 匈牙利算法分配轨迹，并且生成多条csp
    # 目标分配
    matches = np.zeros([n_stage, n_car], dtype=int)        # 目标分配结果
    matches[0, :] = np.arange(n_car)
    for i_stage in range(1, n_stage):
        _, matches[i_stage, ...] = Assign(formation_poses[2*i_stage-2, :], formation_poses[2*i_stage, :])
        matches[i_stage, :] = matches[i_stage ,matches[i_stage-1, :]]


    # matches[n_stage-1, :] = matches[n_stage-1 ,matches[n_stage-2, :]]

    # plt.figure(1)
    # plt.plot(formation_poses[..., 0], formation_poses[..., 1], 'bo')
    # 生成若干条局部轨迹
    # 首先插值补点。保持队形阶段的点。通过队形函数进行修补
    # individual s
    ind_s = np.arange(0, global_csp.s[-1], 0.2)
    ind_s = np.append(ind_s, global_csp.s[-1])
    stages = judge_formation_stage(ind_s, formation_begin_s, ds=ds_trans)

    # 所有稳态阶段的点都加入到局部轨迹中
    ind_s = ind_s[stages>=0]
    ind_s_stage = stages[stages>=0]
    n_s = ind_s.shape[0]
    global_ind_pos = np.array([global_csp.calc_position(s) for s in ind_s])
    plt.figure(1)
    plt.plot(global_ind_pos[:, 0], global_ind_pos[:, 1], 'g*')
    global_ind_yaw = np.array([global_csp.calc_yaw(s) for s in ind_s])
    ind_pos = np.zeros([n_s, n_car, 2])

    for i in range(n_s):
        i_stage = int(ind_s_stage[i])
        ind_pos[i, ...] = gen_formation(global_ind_pos[i, :2], global_ind_yaw[i], n_car, d_car, formation_types[i_stage])
        # 根据目标分配结果重组局部轨迹
        ind_pos[i, ...] = ind_pos[i, matches[i_stage, :], :]
        
    # [TODO]在ind_pos之间的过渡阶段
    new_ind_pos = []
    for i_car in range(n_car):
        ind_pos_icar = copy.deepcopy(ind_pos[:, i_car, :])
        i_point = 0
        for i in range(n_s-1):
            ds = np.linalg.norm(ind_pos[i+1, i_car, :] - ind_pos[i, i_car, :])
            i_point += 1
            if ds >2:
                begin = np.append(ind_pos[i, i_car, :], global_ind_yaw[i])
                end = np.append(ind_pos[i+1, i_car, :], global_ind_yaw[i+1])
                traj = dubins_pf_pro(np.array([begin, end]), len_end_line=0.001)
                traj_np = np.array(traj)
                ind_pos_icar = np.insert(ind_pos_icar, i_point, traj_np[:, :2], axis=0)
                i_point += traj_np.shape[0]
        new_ind_pos.append(ind_pos_icar)
    

    individual_csps =[]
    for i_car in range(n_car):
        individual_csp = Spline2D(new_ind_pos[i_car][: , 0], new_ind_pos[i_car][:, 1] )
        individual_csps.append(individual_csp)
    return global_csp,  individual_csps, matches, ind_pos
    

def ds_trans_init(ds_trans, n_stage, formation_begin_s, s_max):
    ''' 方便初始化队形调整间隔。主要功能：将 -1 转成队形末端-1
    '''
    ds_trans_new = copy.deepcopy(ds_trans)
    if len(ds_trans_new) ==0:
        ds_trans_new = [ds_formation_transition for i in range(n_stage)]      # 队形调整间距
    else:
        if ds_trans_new[-1] == -1:
            ds_trans_new[-1] =  s_max - formation_begin_s[-1]
    return ds_trans_new

def plot_csp(csp: Spline2D, plot_style='r*', ds=1):
    x,y,_,_,_= spline_expand(csp, ds=ds)
    # print(x,y)
    plt.plot(x,y, plot_style)


def dubins_pf_pro(car_poses, ctrl_points=[], r=r_pix, step_size=0.1, len_end_line = 1):
    ''' dubin path finding. 默认控制点插在起点和终点之间。
    Params:
        car_poses: 2*3. 2： 暂时仅支持起点和终点。3: x,y,yaw
        step_size: dubins轨迹采样长度
        len_end_line: 轨迹最末端添加一段直线摆正车身。直线长度。
    '''
    if len(ctrl_points)>0:
        # n_pose_car = car_poses.shape[1]
        ctrl_points = np.array(ctrl_points).reshape([-1])
        car_poses = np.concatenate((car_poses[0, :], ctrl_points, car_poses[1, :])).reshape([-1,3])
    
    # 最后一步矫正加直线的误差。保证停到终点
    theta = car_poses[-1, -1]
    line_vec = np.array([math.cos(theta), math.sin(theta), 0])*len_end_line
    car_poses[-1, :] = car_poses[-1, :] - line_vec

    trajs = []
    for i in range(car_poses.shape[0]-1):
        conf = dubins.shortest_path(car_poses[i, :], car_poses[i+1, :], r)
        traj, s = conf.sample_many(step_size)
        # 转圈检测
        d = np.linalg.norm(car_poses[i+1, :2] - car_poses[i, :2])
        if s[-1] > 4*d:
            print('load task points dubins pf pro error!')
            traj = gen_line(car_poses[i, :], car_poses[i+1, :], dim=3)
        
        trajs.extend(traj)

    # 加一段短直线：保证拐弯后车头朝向正确
    tiny = gen_line(car_poses[-1, :], car_poses[-1, :] + line_vec, ds=0.1, dim=3)
    trajs.extend(tiny)

    trajs = np.array(trajs)

    return trajs


# if __name__ == '__main__':
    # formation_inputs = [0,7, 14, 20]
    # formation_types = [FormationType.Line, FormationType.Triangle, FormationType.Line, FormationType.Line]
    # ds_trans = [ds_formation_transition for i in range(len(formation_inputs))]
    # ds_trans[-1] = 10000
    # plt.figure(1)
    # # plt.plot(center_line[:,0], center_line[:,1], 'bo')

    # global_csp,  individual_csps, matches, formation_poses  = generate_route_with_formation(center_line, formation_inputs, formation_types, n_car, ds_trans=ds_trans)

    # # plot_csp(global_csp,'r.-', ds=0.01)
    # for individual_csp in individual_csps:
    #     plot_csp(individual_csp, 'r*-', ds=1)
    # # for i_car in range(formation_poses.shape[1]):
    # #     plt.plot(formation_poses[:,i_car, 0], formation_poses[:, i_car, 1], 'g*-')
    # plt.axis('square')
    # plt.show()