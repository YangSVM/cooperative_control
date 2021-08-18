#!/usr/bin/env python
import math
import numpy as np 
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from trajectory_tracking.msg import RoadPoint
from trajectory_tracking.msg import Trajectory


M_PI = 3.141593

def NormalizeAngle(angle_rad):
        # to normalize an angle to [-pi, pi]
        a = math.fmod(angle_rad + M_PI, 2.0 * M_PI)
        if a < 0.0:
            a = a + 2.0 * M_PI
        return a - M_PI

class PathPoint:
    def __init__(self, pp_list):
        # pp_list: from CalcRefLine, [rx, ry, rs, rtheta, rkappa, rdkappa]
        self.rx = pp_list[0]
        self.ry = pp_list[1]
        self.rs = pp_list[2]
        self.rtheta = pp_list[3]
        self.rkappa = pp_list[4]
        self.rdkappa = pp_list[5]

class TrajPoint:
    def __init__(self, tp_list):
        # tp_list: from sensors, [x, y, v, a, theta, kappa]
        self.x = tp_list[0]
        self.y = tp_list[1]
        self.v = tp_list[2]
        self.a = tp_list[3]
        self.theta = tp_list[4]
        self.kappa = tp_list[5]

def CartesianToFrenet(path_point, traj_point):
    ''' from Cartesian to Frenet coordinate, to the matched path point
    copy Apollo cartesian_frenet_conversion.cpp'''
    rx, ry, rs, rtheta, rkappa, rdkappa = path_point.rx, path_point.ry, path_point.rs, \
        path_point.rtheta, path_point.rkappa, path_point.rdkappa
    x, y, v, a, theta, kappa = traj_point.x, traj_point.y, traj_point.v, \
        traj_point.a, traj_point.theta, traj_point.kappa
    
    s_condition = np.zeros(3)
    d_condition = np.zeros(3)

    dx = x - rx
    dy = y - ry

    cos_theta_r = math.cos(rtheta)
    sin_theta_r = math.sin(rtheta)

    cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
    d_condition[0] = math.copysign(math.sqrt(dx ** 2 + dy ** 2), cross_rd_nd)

    delta_theta = theta - rtheta
    tan_delta_theta = math.tan(delta_theta)
    cos_delta_theta = math.cos(delta_theta)

    one_minus_kappa_r_d = 1 - rkappa * d_condition[0]
    d_condition[1] = one_minus_kappa_r_d * tan_delta_theta

    kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1]

    d_condition[2] = -kappa_r_d_prime * tan_delta_theta + one_minus_kappa_r_d / (cos_delta_theta ** 2) * \
        (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa)
    
    s_condition[0] = rs
    s_condition[1] = v * cos_delta_theta / one_minus_kappa_r_d

    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
    s_condition[2] = (a * cos_delta_theta - s_condition[1] ** 2 * \
        (d_condition[1] * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d
    
    return s_condition, d_condition

def FrenetToCartesian(path_point, s_condition, d_condition):
    ''' from Frenet to Cartesian coordinate
    copy Apollo cartesian_frenet_conversion.cpp'''
    rx, ry, rs, rtheta, rkappa, rdkappa = path_point.rx, path_point.ry, path_point.rs, \
        path_point.rtheta, path_point.rkappa, path_point.rdkappa
    if math.fabs(rs - s_condition[0]) >= 1.0e-6:
        pass
        # print("the reference point s and s_condition[0] don't match")
    
    cos_theta_r = math.cos(rtheta)
    sin_theta_r = math.sin(rtheta)

    x = rx - sin_theta_r * d_condition[0]
    y = ry + cos_theta_r * d_condition[0]

    one_minus_kappa_r_d = 1 - rkappa * d_condition[0]
    tan_delta_theta = d_condition[1] / one_minus_kappa_r_d
    delta_theta = math.atan2(d_condition[1], one_minus_kappa_r_d)
    cos_delta_theta = math.cos(delta_theta)
    theta = NormalizeAngle(delta_theta + rtheta)

    kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1]
    kappa = ((d_condition[2] + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta ** 2 / one_minus_kappa_r_d \
        + rkappa) * cos_delta_theta / one_minus_kappa_r_d
    
    d_dot = d_condition[1] * s_condition[1]
    v = math.sqrt((one_minus_kappa_r_d * s_condition[1]) ** 2 + d_dot ** 2)

    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
    a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta + s_condition[1] ** 2 / cos_delta_theta * \
        (d_condition[1] * delta_theta_prime - kappa_r_d_prime)

    tp_list = [x, y, v, a, theta, kappa]
    return TrajPoint(tp_list)

def CalcRefLine(cts_points):
    ''' deal with reference path points 2d-array
    
    to calculate rs/rtheta/rkappa/rdkappa according to cartesian points'''
    rx = cts_points[0]      # the x value
    ry = cts_points[1]      # the y value
    rs = np.zeros_like(rx)
    rtheta = np.zeros_like(rx)
    rkappa = np.zeros_like(rx)
    rdkappa = np.zeros_like(rx)
    for i, x_i in enumerate(rx):
        y_i = ry[i]
        if i != 0:
            dx = rx[i] - rx[i-1]
            dy = ry[i] - ry[i-1]
            rs[i] = rs[i-1] + math.sqrt(dx ** 2 + dy ** 2)
        if i < len(ry)-1:
            dx = rx[i+1] - rx[i]
            dy = ry[i+1] - ry[i]
            ds = math.sqrt(dx ** 2 + dy ** 2)
            rtheta[i] = math.copysign(math.acos(dx / ds), dy)
    rtheta[-1] = rtheta[-2]
    rkappa[:-1] = np.diff(rtheta) / np.diff(rs)
    rdkappa[:-1] = np.diff(rkappa) / np.diff(rs)
    rkappa[-1] = rkappa[-2]
    rdkappa[-1] = rdkappa[-3]
    rdkappa[-2] = rdkappa[-3]
    path_points = []
    for i in range(len(rx)):
        path_points.append(PathPoint([rx[i], ry[i], rs[i], rtheta[i], rkappa[i], rdkappa[i]]))
    return path_points

def LinearInterpolate(path_point_0, path_point_1, rs_inter):
    ''' path point interpolated linearly according to rs value
    path_point_0 should be prior to path_point_1'''
    def lerp(x0, x1, w):
        return x0 + w * (x1 - x0)
    def slerp(a0, a1, w):
        # angular, for theta
        a0_n = NormalizeAngle(a0)
        a1_n = NormalizeAngle(a1)
        d = a1_n - a0_n
        if d > M_PI:
            d = d - 2 * M_PI
        elif d < -M_PI:
            d = d + 2 * M_PI
        a = a0_n + w * d
        return NormalizeAngle(a)
    rs_0 = path_point_0.rs
    rs_1 = path_point_1.rs
    weight = (rs_inter - rs_0) / (rs_1 - rs_0)
    if weight < 0 or weight > 1:
        print("weight error, not in [0, 1]")
        quit()
    rx_inter = lerp(path_point_0.rx, path_point_1.rx, weight)
    ry_inter = lerp(path_point_0.ry, path_point_1.ry, weight)
    rtheta_inter = slerp(path_point_0.rtheta, path_point_1.rtheta, weight)
    rkappa_inter = lerp(path_point_0.rkappa, path_point_1.rkappa, weight)
    rdkappa_inter = lerp(path_point_0.rdkappa, path_point_1.rdkappa, weight)
    return PathPoint([rx_inter, ry_inter, rs_inter, rtheta_inter, rkappa_inter, rdkappa_inter])

def MatchPoint(traj_point, path_points):
    ''' find the closest/projected point on the reference path
    the deviation is not large; the curvature is not large'''
    def DistSquare(traj_point, path_point):
        dx = path_point.rx -  traj_point.x
        dy = path_point.ry -  traj_point.y
        return (dx ** 2 + dy ** 2)
    dist_all = []
    for path_point in path_points:
        dist_all.append(DistSquare(traj_point, path_point))
    dist_min = DistSquare(traj_point, path_points[0])
    index_min = 0
    for index, path_point in enumerate(path_points):
        dist_temp = DistSquare(traj_point, path_point)
        if dist_temp < dist_min:
            dist_min = dist_temp
            index_min = index
    path_point_min = path_points[index_min]
    if index_min == 0 or index_min == len(path_points):
        return path_point_min
    else:
        path_point_next = path_points[index_min + 1]
        path_point_last = path_points[index_min - 1]
        vec_p2t = np.array([traj_point.x - path_point_min.rx, traj_point.y - path_point_min.ry])
        vec_p2p_next = np.array([path_point_next.rx - path_point_min.rx, path_point_next.ry - path_point_min.ry])
        vec_p2p_last = np.array([path_point_last.rx - path_point_min.rx, path_point_last.ry - path_point_min.ry])
        if np.dot(vec_p2t, vec_p2p_next) >= 0:
            rs_inter = path_point_min.rs + np.dot(vec_p2t, vec_p2p_next / np.linalg.norm(vec_p2p_next))
            return LinearInterpolate(path_point_min, path_point_next, rs_inter)
        else:
            rs_inter = path_point_min.rs - np.dot(vec_p2t, vec_p2p_last / np.linalg.norm(vec_p2p_last))
            return LinearInterpolate(path_point_last, path_point_min, rs_inter)

class PolyTraj:
    def __init__(self, s_cond_init, d_cond_init, total_t):
        self.s_cond_init = s_cond_init
        self.d_cond_init = d_cond_init
        self.total_t = total_t          # to plan how long in seconds
        self.delta_s = 0

    def __QuinticPolyCurve(self, y_cond_init, y_cond_end, x_dur):
        ''' form the quintic polynomial curve: y(x) = a0 + a1 * delta_x + ... + a5 * delta_x ** 5, x_dur = x_end - x_init
        y_cond = np.array([y, y', y'']), output the coefficients a = np.array([a0, ..., a5])'''
        a0 = y_cond_init[0]
        a1 = y_cond_init[1]
        a2 = 1.0 / 2 * y_cond_init[2]
        
        T = x_dur
        h = y_cond_end[0] - y_cond_init[0]
        v0 = y_cond_init[1]
        v1 = y_cond_end[1]
        acc0 = y_cond_init[2]
        acc1 = y_cond_end[2]
        a3 = 1.0 / (2 * T ** 3) * (20 * h - (8 * v1 + 12 * v0) * T - (3 * acc0 - acc1) * T ** 2)
        a4 = 1.0 / (2 * T ** 4) * (-30 * h + (14 * v1 + 16 * v0) * T + (3 * acc0 - 2 * acc1) * T ** 2)
        a5 = 1.0 / (2 * T ** 5) * (12 * h - 6 * (v1 + v0) * T + (acc1 - acc0) * T ** 2)
        return np.array([a0, a1, a2, a3, a4, a5])

    def GenLongTraj(self, s_cond_end):
        self.long_coef =  self.__QuinticPolyCurve(self.s_cond_init, s_cond_end, self.total_t)
        self.delta_s = self.long_coef[1] * self.total_t + self.long_coef[2] * self.total_t ** 2 + \
            self.long_coef[3] * self.total_t ** 3 + self.long_coef[4] * self.total_t ** 4 + \
                self.long_coef[5] * self.total_t ** 5
        # return self.long_coef
    
    def GenLatTraj(self, d_cond_end):
        # GenLatTraj should be posterior to GenLongTraj
        self.lat_coef = self.__QuinticPolyCurve(self.d_cond_init, d_cond_end, self.delta_s)
        # return self.lat_coef
    
    def GenCombinedTraj(self, path_points, delta_t):
        ''' combine long and lat traj together
        
        F2C function is used to output future traj points in a list to follow'''
        a0_s, a1_s, a2_s, a3_s, a4_s, a5_s = self.long_coef[0], self.long_coef[1], self.long_coef[2], \
            self.long_coef[3], self.long_coef[4], self.long_coef[5]
        a0_d, a1_d, a2_d, a3_d, a4_d, a5_d = self.lat_coef[0], self.lat_coef[1], self.lat_coef[2], \
            self.lat_coef[3], self.lat_coef[4], self.lat_coef[5]
        
        rs_pp_all = []              # the rs value of all the path points
        for path_point in path_points:
            rs_pp_all.append(path_point.rs)
        rs_pp_all = np.array(rs_pp_all)
        num_points = int(math.floor(self.total_t / delta_t))
        s_cond_all = []             # possibly useless
        d_cond_all = []             # possibly useless
        pp_inter = []               # possibly useless
        tp_all = []                 # all the future traj points in a list
        t, s = 0, 0                 # initialize variables, s(t), d(s) or l(s)
        for i in range(num_points):
            s_cond = np.zeros(3)
            d_cond = np.zeros(3)

            t = t + delta_t
            s_cond[0] = a0_s + a1_s * t + a2_s * t ** 2 + a3_s * t ** 3 + a4_s * t ** 4 + a5_s * t ** 5
            s_cond[1] = a1_s + 2 * a2_s * t + 3 * a3_s * t ** 2 + 4 * a4_s * t ** 3 + 5 * a5_s * t ** 4
            s_cond[2] = 2 * a2_s + 6 * a3_s * t + 12 * a4_s * t ** 2 + 20 * a5_s * t ** 3
            s_cond_all.append(s_cond)

            s = s_cond[0] - a0_s
            d_cond[0] = a0_d + a1_d * s + a2_d * s ** 2 + a3_d * s ** 3 + a4_d * s ** 4 + a5_d * s ** 5
            d_cond[1] = a1_d + 2 * a2_d * s + 3 * a3_d * s ** 2 + 4 * a4_d * s ** 3 + 5 * a5_d * s ** 4
            d_cond[2] = 2 * a2_d + 6 * a3_d * s + 12 * a4_d * s ** 2 + 20 * a5_d * s ** 3
            d_cond_all.append(d_cond)

            index_min = np.argmin(np.abs(rs_pp_all - s_cond[0]))
            path_point_min = path_points[index_min]
            if index_min == 0 or index_min == len(path_points):
                path_point_inter = path_point_min
            else:
                if s_cond[0] >= path_point_min.rs:
                    path_point_next = path_points[index_min + 1]
                    path_point_inter = LinearInterpolate(path_point_min, path_point_next, s_cond[0])
                else:
                    path_point_last = path_points[index_min - 1]
                    path_point_inter = LinearInterpolate(path_point_last, path_point_min, s_cond[0])
            pp_inter.append(path_point_inter)

            traj_point = FrenetToCartesian(path_point_inter, s_cond, d_cond)
            tp_all.append(traj_point)
        return tp_all 

def talker(tp_list):
    pub = rospy.Publisher('local_trajectory', Trajectory, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz

    while not rospy.is_shutdown():

        local_trajectory = Trajectory()
        for i in range(len(tp_list)):
            tp = tp_list[i]
            road_point = RoadPoint()
            road_point.x = tp[0]   # x
            road_point.y = tp[1]   # y
            road_point.v = tp[2]    #v
            road_point.a= tp[3] # a
            road_point.yaw = tp[5]    #theta heading
            road_point.kappa = tp[4]    # kappa
            local_trajectory.roadpoints.append(road_point)

        # rospy.loginfo(msg)
        pub.publish(local_trajectory)
        rate.sleep()

if __name__ == '__main__':
    ref_data = np.loadtxt('/home/tiecun/catkin_ws/src/MA-L5-THICV/trajectory_tracking/gnss_gpchc_driver/map/roadMap_lzjSouth1.txt')
    rx = ref_data[:, 1]
    ry = ref_data[:, 2]	
    cts_points = np.array([rx, ry])
    path_points = CalcRefLine(cts_points)
    tp_list = [-252, -503, 1, 0, -M_PI/4, 0]
    traj_point = TrajPoint(tp_list)
    matched_point = MatchPoint(traj_point, path_points)

    s_cond_init, d_cond_init = CartesianToFrenet(matched_point, traj_point)
    total_t = 36
    poly_traj = PolyTraj(s_cond_init, d_cond_init, total_t)
    s_cond_end = np.array([s_cond_init[0] + 10, 0, 0])
    poly_traj.GenLongTraj(s_cond_end)
    d_cond_end = np.array([0, 0, 0])
    poly_traj.GenLatTraj(d_cond_end)
    delta_t = 0.1 * 1
    tp_all = poly_traj.GenCombinedTraj(path_points, delta_t)
    tp_list = []
    for tp in tp_all:
	    tp_list.append([tp.x,tp.y,tp.v,tp.a,tp.theta,tp.kappa])
    try:
        talker(tp_list)
    except rospy.ROSInterruptException:
        pass
