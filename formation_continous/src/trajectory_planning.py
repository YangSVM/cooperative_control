#!/usr/bin/env python3

"""
单车轨迹规划

采样方式：
    - 横向终点位置采样(采样间距+车宽 是车能通过的最大间隙)
    - 终点时间采样：多久能够回到中心线
    - 终点纵向速度采样：达到终点后，车的速度是多少。
分别对横纵向用5次、3次多项式(关于时间t)，对起点状态和末端状态列方程，可以求解该轨迹。

"""

from typing import List
import numpy as np
import copy
import math


from math import pi
from formation_common.formation_zoo import *


from formation_common.polynomials_planner import QuarticPolynomial, CubicPolynomial4LaneChange
from formation_common.cubic_spline_planner import Spline2D, cartesian2frenet
from trajectory_planning_config import *


class FrenetPath:
    ''' 结构体。每条采样的轨迹。
    '''
    def __init__(self):
        self.t = []
        self.d = []
        self.s = []

        self.x = []
        self.y = []
        self.yaw = []

        self.v = []

    def to_numpy(self)->np.ndarray:
        x, y, v = self.x, self.y, self.v
        min_len = min([len(x), len(y), len(v)])
        x = x[:min_len]
        y = y[:min_len]
        v = v[:min_len]
        traj_np = np.array([x , y, v]).T
        return traj_np
    
    def set_v(self, v):
        x, y= self.x, self.y
        min_len = min([len(x), len(y)])
        self.v = [v for i in range(min_len)]


def calc_frenet_paths(c_speed, c_d,  s0):
    frenet_paths = []

    Ti = T_PLANNING
    fp = FrenetPath()

    lat_qp = CubicPolynomial4LaneChange(c_d,   0, 0.0, 0.0, Ti)
    # 让s最大为终点
    fp.t = [t for t in np.arange(0.0, Ti, DT)]
    fp.d = [lat_qp.calc_point(t) for t in fp.t]


    tv = TARGET_SPEED
    tfp = copy.deepcopy(fp)
    lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

    tfp.s = [lon_qp.calc_point(t) for t in fp.t]

    frenet_paths.append(tfp)

    return frenet_paths




def calc_global_paths(fplist, csp):
    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
        if len(fp.yaw) !=0:
            fp.yaw.append(fp.yaw[-1])

    return fplist


def trajectory_planning(csp: Spline2D, pose, ob=[])->FrenetPath:
    ''' frenet坐标系规划。删除采样。没有避障功能。
    Params:

    '''
    s0,  c_d = cartesian2frenet(pose[0], pose[1], csp)
    isGoal = False
    if abs(s0 - csp.s[-1]) < 0.1:
        isGoal = True
    c_speed = TARGET_SPEED
    fplist = calc_frenet_paths(c_speed, c_d, s0)
    fplist = calc_global_paths(fplist, csp)

    return fplist[0], isGoal


def main():
    return 0


if __name__ == '__main__':
    main()
