#!/usr/bin/env python3
'''辅助设计全局轨迹
'''
import numpy as np
import math

from numpy.linalg import linalg
PI = np.pi

def gen_line(begin, end, ds=0.01, dim=2):
    ''' 生成一条起点(x,y)到终点(x,y)的直线点。距离间隔为ds
    '''
    begin = np.array(begin)
    end = np.array(end)
    vec = end - begin
    vec_len = np.linalg.norm(vec[..., :2])
    n_points = int(vec_len/ds)
    line = np.zeros([n_points, 3])
    line[:, 0] = np.linspace(begin[0], end[0], n_points)
    line[:, 1] = np.linspace(begin[1], end[1], n_points)
    line[:, 2] = math.atan2(vec[1], vec[0])
    if dim == 2:
        line = line[:, :2]
        
    return line


def gen_arc(center, theta_s, theta_e, r, isCounterClock=1, ds=0.1):
    ''' 生成一条圆弧
    Params:
        theta_s: start. 角度制
        theta_e: end. 角度制
        isCounterClock: 逆时针为正。否则为负。
        ds: 点列距离
    Return:
        np.array. [n, 3]: x,y,v
    '''
    if isCounterClock==0:
        isCounterClock=-1
    theta = np.arange(theta_s/180*PI, theta_e/180*PI, isCounterClock*5*ds/r)       # 保证大概1cm一个点
    roadpoints = np.empty([len(theta), 2])
    roadpoints[:,0] = r*np.cos(theta) + center[0]
    roadpoints[:,1] = r*np.sin(theta) + center[1]

    return roadpoints

def del_duplecate_points(rps):
    ''' 点列中相近的相邻元素。
    '''
    rps = np.array(rps)
    d_vec = rps[1:, :] - rps[:-1, :]
    d = np.linalg.norm(d_vec, axis=1)
    d = np.insert(d, 0, 100)
    rps_new = rps[d>1e-4, :]
    
    return rps_new