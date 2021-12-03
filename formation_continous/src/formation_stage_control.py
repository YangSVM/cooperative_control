#!/usr/bin/env python3

"""
v 1.0
编队状态控制器
"""
import matplotlib.pyplot as plt
from formation_common.cubic_spline_planner import Spline2D, spline_expand 
from formation_common.global_planning import generate_route_with_formation, ds_trans_init
from typing import List
from formation_common.formation_core import assgin_traj, judge_formation_end, judge_formation_stage, Assign
import numpy as np
from formation_common.config_formation_continous import *

# 队形变换相关参数
class FormationStage():
    ''' 用于判断队列处于哪个状态(FormationState)。每个状态都是存在一个区间中[formation_begin_s, formation_begin_s+ds_trans]
    以第一条轨迹的s为准。
    '''
    def __init__(self, individual_csps, formation_begin_s, formation_types, ds_trans, matches=[]) -> None:
        ''' 具有要素
        '''
        self.individual_csps, self.formation_begin_s, self.formation_types, self.ds_trans =\
                    individual_csps, formation_begin_s, formation_types, ds_trans
        self.d_car = d_car
        self.matches = np.array(matches, dtype=int)
        
        # self.initial()
        # 队形变换阶段，调整避免冲突。delta_s_remain越多，速度提升越快
        self.ddelta_s=[]
        self.v= TARGET_SPEED
        self.priority = []     # 强制执行的通行次序
    
    @classmethod
    def from_individual(cls, individuals:List[np.ndarray], formation_begin_s, formation_types, ds_trans, matches=[]):
        ''' 从各车轨迹创建. 
        Params:
            individuals: n_car. ndarray: [n_points, 2]
        '''
        individual_csps = [Spline2D(pos[:, 0], pos[:, 1]) for pos in individuals]
        return cls( individual_csps, formation_begin_s, formation_types, ds_trans, matches=matches)

    @classmethod
    def from_global(cls, center_line, n_car, formation_begin_s, formation_types, d_car=2.5,ds_trans=[], ):
        ''' 从头车轨迹创建
        '''
        global_csp,  individual_csps, matches, formation_poses  = \
                generate_route_with_formation(center_line, formation_begin_s, formation_types, n_car, d_car=d_car,ds_trans=ds_trans)
        
        
        # if center_line.shape[0] ==14:
        if isPlot:
            # plt.clf()
            plt.axis('equal')

            plt.plot(center_line[:,0], center_line[:,1],'g*-')

            x,y, _ ,_,_= spline_expand(global_csp)
            plt.plot(x,y, 'k*-')

            for i, csp_i in enumerate(individual_csps):
                x,y, _ ,_,_= spline_expand(csp_i)
                plt.plot(x,y, 'r*-')
            plt.axis('equal')

        return cls( individual_csps, formation_begin_s, formation_types, ds_trans, matches=matches)

    def initial(self, leader_id):
        self.leader_id = leader_id
        
        # ds_trans 初始化
        delta_s_end = 0.1         # 当处于这个阶段内。
        self.ds_trans = ds_trans_init(self.ds_trans, len(self.formation_begin_s), self.formation_begin_s, self.individual_csps[self.leader_id].s[-1] - delta_s_end)

    def judge_formation_stage(self, s_leader):
        stage = judge_formation_stage([s_leader], self.formation_begin_s, self.ds_trans)[0]
        return stage

    def get_end_position(self):
        n_csps = len(self.individual_csps)
        formation_np = np.ones([n_csps, 2])
        for i in range(n_csps):
            s = self.individual_csps[i].s[-1] - 0.1
            formation_np[i, :] = self.individual_csps[i].calc_position(s)
        return formation_np


if __name__ == '__main__':
    pass
