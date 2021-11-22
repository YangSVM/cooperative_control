#!/usr/bin/env python3

"""
Author: Shi Gongchen, Yang Yibin
在右手系中完成
"""

import rospy
import numpy as np
import copy
import math
from math import pi
import time
from formation_common.formation_zoo import *
from scipy.optimize import linear_sum_assignment
from rospy.names import canonicalize_name
from trajectory_tracking.msg import Trajectory, RoadPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from enum import Enum

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise



# Parameter
MAX_SPEED = 10.8 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.2  # time tick [s]
# DT = 1
MAX_T = 10.0  # max prediction time [s]
MIN_T = 4.0  # min prediction time [s]
TARGET_SPEED = 3 # target speed [m/s]
D_T_S = TARGET_SPEED / 5  # target speed sampling length [m/s]
# N_S_SAMPLE = 1  # sampling number of target speed
N_S_SAMPLE = 10  # sampling number of target speed

ROBOT_RADIUS = 1.0  # robot radius [m]
t_thre=1.5 #碰撞检测时间阈值[s]

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 10
K_LAT = 1.0
K_LON = 3.0

show_animation = False

id_real = [2, 5]
#队列中车辆的状态：
FormationState = Enum('FormationState', ('transition', 'temp_goal', 'goal'))

class FormationROS():
    def __init__(self, n_car, id_list = [1,2,5], id_id_list_real=[1,2]) -> None:
        rospy.init_node("formation_decenteralized")
        self.n_car = n_car
        # 调试的是第几辆车

        self.id_list = id_list
        self.id_id_list_real = id_id_list_real   # id_list中，第几辆车是真车，其余车是虚拟车。即id_list[1],id_list[2](2,5号车)是真车
        # 发布车辆轨迹
        self.pubs = [rospy.Publisher('car'+str(id_list[i])+'/local_trajectory', Trajectory, queue_size=1) for i in range(n_car)]
        self.pub_csps = [rospy.Publisher('car'+str(id_list[i])+'/global_trajectory', Trajectory, queue_size=1) for i in range(n_car)]
        self.pub_wp =rospy.Publisher('/temp_goal', Trajectory, queue_size=1)
        # 接收所有车辆的信息
        for i in range(n_car):
            rospy.Subscriber('car'+str(id_list[i])+'/gps', Odometry,  self.sub_gps_states, i)

        # 车辆状态
        self.pose_states = [Pose() for i in range(n_car)]
        self.gps_flag = [False for i in range(n_car)]

        self.rate = rospy.Rate(10)

    def sub_gps_states(self, msg, i):
        self.gps_flag[i] = True
        self.pose_states[i].position.x = msg.pose.pose.position.x
        self.pose_states[i].position.y = msg.pose.pose.position.y

        self.pose_states[i].orientation.z = msg.twist.twist.angular.z

    def get_pose_states(self):
        if all([self.gps_flag[x] for x in self.id_id_list_real] ) == True:
            return self.pose_states
        else:
            rospy.logwarn('gps info not ready!')
            print(self.gps_flag)
            return None

    def publish(self, v_path_x, v_path_y, v_speed ):
        '''在正常坐标系下规划，轨迹跟踪是左手系下的，需要转换
        '''
        n_car = self.n_car
        for i in range(n_car):
            n_road_point = len(v_path_x[i])
            traj = Trajectory()
            for i_rp in range(n_road_point):
                rp = RoadPoint()
                rp.x = v_path_x[i][i_rp]        # 注意x = y的转换
                rp.y = v_path_y[i][i_rp]
                rp.v = v_speed[i][i_rp]
                traj.roadpoints.append(rp)
            self.pubs[i].publish(traj)

    def publish_csps(self, trajs):
        '''在正常坐标系下规划，轨迹跟踪是左手系下的，需要转换
        '''
        n_car = self.n_car
        for i_car in range(n_car):
            self.pub_csps[i_car].publish(trajs[i_car])

    def publish_wp(self, trajs):
        self.pub_wp(trajs)

def generate_traj_by_csp(csps :cubic_spline_planner.Spline2D):
    ''' cubic_spline_planner.Spline2D ->  trajectory_tracking.msg  Trajectory
    '''
    n_car = len(csps)
    trajs = []
    for i_car in range(n_car):
        csp = csps[i_car]
        s = np.arange(0, csp.s[-1], 0.1)
        n_point = len(s)
        csp_pos = np.zeros([n_point, 2])
        for i in range(n_point):
            csp_pos[i, 0], csp_pos[i, 1] = csp.calc_position(s[i])
        traj = Trajectory()
        for i_rp in range(n_point):
            rp = RoadPoint()
            rp.x = csp_pos[i_rp, 0]       
            rp.y = csp_pos[i_rp, 1]
            traj.roadpoints.append(rp)
        trajs.append(traj)
    return trajs

def generate_traj_by_wp(wpx, wpy):
    trajs = []
    n_car  = wpx.shape[0]
    n_point = wpx.shape[1]
    for i in range(n_point):
        traj = Trajectory()
        for i_car in range(n_car):
            rp = RoadPoint()
            rp.x = wpx[i_car, i]
            rp.y = wpy[i_car, i]
            traj.roadpoints.append(rp)
        trajs.append(traj)
    return trajs

class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    # 横向位置偏移采样
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        # 删除该采样
        for Ti in np.arange(MIN_T, MAX_T, 1):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            # 让s最大为终点
            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            # 纵向速度采样。时间给定，所以可以急停刹车。(是否真的需要速度规划呢？后面又有速度规划)
            # for tv in np.arange(0, TARGET_SPEED + D_T_S, D_T_S):
            for tv in [TARGET_SPEED]:
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


# 给出参考轨迹csp, 求解对应frenet坐标系下的s-d
def xy2frenet(csp :cubic_spline_planner.Spline2D, pos_ros):
    pos = np.array([pos_ros.x, pos_ros.y])

    s = np.arange(0, csp.s[-1], 0.1)
    n_point = len(s)
    csp_pos = np.zeros([n_point, 2])
    for i in range(n_point):
        csp_pos[i, 0], csp_pos[i, 1] = csp.calc_position(s[i])
    pos_delta = csp_pos - pos
    d = (pos_delta[:,0]**2 + pos_delta[:,1]**2)**0.5
    d_min = d.min()
    d_error = 0.1
    idx = np.where(d < d_min + d_error)
    id = idx[0][-1]
    s_res = s[id]
    d_res = np.linalg.norm(csp_pos[id,:]-pos)

    # 增加判断d的正负
    if id+1 < n_point:
        vec  =  csp_pos[id+1, :] - csp_pos[id, :]
    else:
        vec = csp_pos[id, :] -csp_pos[id-1, :]
    # 法向量
    vec_normal  = [0, 0]
    vec_normal[0], vec_normal[1] = -vec[1] , vec[0]
    pos_vec = pos - csp_pos[id,:]
    pos_sign = np.sign(np.dot(pos_vec, vec_normal))

    # if s_res >0.5:
    #     print('error xy2frenet: ', s_res)
    return s_res, d_res*pos_sign


# def odometry2frenet(cps, odom):


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
            fp.ds.append(math.hypot(dx, dy))
        if len(fp.yaw) !=0:
            fp.yaw.append(fp.yaw[-1])
        if len(fp.ds) != 0:
            fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        # elif any([abs(c) > MAX_CURVATURE for c in
                #   fplist[i].c]):  # Max curvature check
            # continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    # t1 = time.time()
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    t2 = time.time()
    # print(t2-t1)
    fplist = calc_global_paths(fplist, csp)
    # t3 = time.time()

    if fplist is None:
        print('none fplist')

    fplist = check_paths(fplist, ob)
    # t4 = time.time()
    # print(t3-t2, ' ', t4-t3)
    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


# 碰撞检测2
# 先假设匀速运动
def collision_avoid(local_trajs):
    x, y, v=[],[],[]
    n_car = len(local_trajs)
    for i in range(n_car):
        x.append(np.array(local_trajs[i][0]))
        y.append(np.array(local_trajs[i][1]))
        v.append(np.array(local_trajs[i][2]))

    # 取x中最短的
    t_min = min([len(xi) for xi in x])

    ins_condition=[]  # 存储插入情况的列表，每个元素的形式为[j,ind_s,ind_e]

    # 对每个时间点进行碰撞检测
    for ts in range(t_min):
        for i in range(n_car-1):
            for j in range(i+1, n_car):
                d_ij = np.hypot(x[i][ts]-x[j][ts], y[i][ts]-y[j][ts])
                if d_ij < ROBOT_RADIUS*2:
                    ins_condition.append([i, j, ts])
                    v[i][ts:] = 0
                    rospy.logwarn('car waiting' + str(i))
                    continue

    # 碰撞点t_thre进行停车。速度置为0.x,y不变
    # for con
    # v[ts:] = 0
    return x,y,v


# 进行目标分配的函数，并变换终点坐标以使编号保持一致
def Assign(vp_x,vp_y,target_x,target_y):
    n_car=len(vp_x)
    cost_matrix = np.zeros((n_car, n_car))  # 代价矩阵
    for i in range(n_car):
        for j in range(n_car):
            sq=(target_x[j]-vp_x[i])**2+(target_y[j]-vp_y[i])**2
            cost_matrix[i, j] = sq

    matches_i, matches_j = linear_sum_assignment(cost_matrix)

    print("cost_matrix=")
    print(cost_matrix)
    print("assignment result=")
    print(matches_i, matches_j)
    target_xx,target_yy=[0]*len(target_x),[0]*len(target_y)

    for mi, mj in zip(matches_i, matches_j):
        target_xx[mi] = target_x[mj]
        target_yy[mi] = target_y[mj]

    return target_xx,target_yy


def zip_vehcle_state(s0, c_speed, c_d, c_d_d, c_d_dd):
    states = [s0, c_speed, c_d, c_d_d, c_d_dd]
    return states


def unzip_vehicle_state(states):
    s0, c_speed, c_d, c_d_d, c_d_dd = states[0], states[1], states[2], states[3], states[4]
    return s0, c_speed, c_d, c_d_d, c_d_dd



def local_traj_gen(wp_x,wp_y,ob, formation_ros: FormationROS, origin_shift=[0,0]):
    ''' 计算各车路径的函数，输入为全局路径需要经过的路点(每个路点为队形对应点)、障碍物位置
    params:
        origin_shift: [x, y]. 原点偏移矫正。将所有输入都增加该原点偏移量。
    '''
    wp_x = np.array(wp_x).T
    # wp_x[:,1:] = wp_x[:,1:] + origin_shift[0]
    wp_y = np.array(wp_y).T
    # wp_y[:,1:] = wp_y[:,1:] + origin_shift[1]
    # ob = ob + origin_shift

    # wp_x :  n_car*n_x
    path_x, path_y,path_v = [], [], []  # 各车依次路径点的x，y坐标及速度
    csps  = []
    n_car = wp_x.shape[0]

    # 全局路径规划，不考虑障碍物
    for i in range(n_car):
        t1 = time.time()
        tx, ty, tyaw, tc, csp = generate_target_course(wp_x[i, :], wp_y[i, :])
        csps.append(csp)
        t2 = time.time()
        print('全局路径规划时间: ', t2-t1)
        # formation_ros.publish(tx,ty)
        # '全局路径规划时间: ', 0.008975505828857422s
    trajs = generate_traj_by_csp(csps)
    wp_traj = generate_traj_by_wp(wp_x, wp_y)
    formation_ros.pub_wp(wp_traj)
    # 局部路径规划

    v_x, v_y, v_s= [], [],[]
    # area = 20.0  # animation area length [m]
    c_speed = 3.6 / 3.6  # current speed [m/s]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    # 存储每辆车的状态
    state0 = zip_vehcle_state(s0, c_speed, c_d, c_d_d, c_d_dd)
    states = [state0 for i in range(n_car)]

    # 局部路径，用于存放v_x, v_y, v_s（横纵坐标速度）
    temp_local_trajs = [[[],[],[]] for i in range(n_car)]           # 规划的轨迹
    paths = [[] for i in range(n_car)]                                          # 当前规划的轨迹
    # 每辆车是否都已经到达终点
    isGoal = [False for i in range(n_car)]
    formation_state  = [FormationState.transition for i in range(n_car)]
    last_trajetory = None

    # 全局状态管理器：[起点，第一个队形点)为阶段0，[第一个队形点，第二个队形点)为阶段1，以此类推
    vehicle_formation_stage = [0 for i in range(n_car)]

    while(True):
        # 1. 首先规划出3条路径

        states = formation_ros.get_pose_states()
        if states is None:
            time.sleep(5)           # waiting for gnss message
            continue

        for i in range(n_car):
            csp = csps[i]
            s, d= xy2frenet(csp, states[i].position)

            # 判断是否到达终点
            end_x = csp.sx.y[-1]
            end_y = csp.sy.y[-1]
            if np.hypot(states[i].position.x - end_x, states[i].position.y - end_y) <= 0.5:
                print("Goal", np.hypot(states[i].position.x - end_x, states[i].position.y - end_y))
                isGoal[i] = True
                continue
            
            # 判断是否到达队形点
            for i_wp in range(1, wp_x.shape[1] -1):
                d2temp_goal = np.hypot(states[i].position.x - wp_x[i, i_wp], states[i].position.y - wp_y[i, i_wp])
                if d2temp_goal<2:
                    print('car', i,': temp goal. Stage', i_wp)
                    vehicle_formation_stage[i] = i_wp

            isReady4Next = True
            # 如果三辆车都达到队形点，则进入过度期间继续下一阶段队形变换，否则停车等待
            for i_car in range(n_car):
                if i_car == i or i_car not in formation_ros.id_id_list_real:
                    # 若为自车或者虚拟车，不判断
                    continue
                if vehicle_formation_stage[i] >  vehicle_formation_stage[i_car]:
                    # 如果有比他慢的车
                    isReady4Next  = False
            if isReady4Next:
                # rospy.loginfo('is ready for next transition')
                formation_state = [FormationState.transition for i in range(n_car)]
            else:
                print('car', i,': temp goal. waiting')
                if last_trajetory is None:
                    rospy.logwarn('last trajectory is None')
                    continue
                path_x, path_y, path_v = last_trajetory 
                tmp_v = path_v[i][:]
                path_v[i][:]= np.zeros(tmp_v.shape)
                temp_local_trajs[i][0] = path_x[i][:]
                temp_local_trajs[i][1] = path_y[i][:]
                temp_local_trajs[i][2] = path_v[i][:]
                continue
            
            rospy.loginfo('car'+str(i) +' formation stage:' + str( vehicle_formation_stage[i]))
            
            s_d = 3/3.6
            path = frenet_optimal_planning(csp, s, s_d, d, 0, 0, ob)

            if path is None:
                print('cannot plannning trajectory')
                break
            # 存储单车的路径
            temp_local_trajs[i][0] = path.x
            temp_local_trajs[i][1] = path.y
            temp_local_trajs[i][2] = path.s_d
            paths[i] = path

        # 2. 速度规划。考虑车辆之间避撞
        path_x, path_y, path_v = collision_avoid(temp_local_trajs)
        last_trajetory = path_x, path_y, path_v

        # 3. 发布速度
        formation_ros.publish(path_x, path_y, path_v )
        formation_ros.publish_csps(trajs)
        formation_ros.rate.sleep()
 
    return 0


def test_none_path():
    # 障碍物坐标
    ob = np.array([
                   [10,3],
                   [10,5],
                   [30,3],
                   [30,5],
                   [30,11]
                   ])
    s = 9.6
    d = 0.36223749539638794
    s_d = 3/3.6
    wp_x = np.array([[ 0.        ,  0.        ,  0.        ], [15.        , 12.40192379, 17.59807621],       [30.        , 30.        , 30.        ]])
    wp_y = np.array([[ 0. ,  3. ,  6. ],       [ 9. , 13.5, 13.5],       [ 7. , 13. , 10. ]])
    csps = []
    for i in range(3):
        t1 = time.time()
        tx, ty, tyaw, tc, csp = generate_target_course(wp_x[:, i], wp_y[:, i])
        csps.append(csp)

    csp = csps[2]
    path = frenet_optimal_planning(csp, s, s_d, d, 0, 0, ob)


def main():

    print(__file__ + " start!!")

    # 全局变量
    n_car=3

    # ros init
    formation_ros = FormationROS(n_car)
  
    # ros init end    
    origin_shift = [-85, -7]

    # 队形1。车辆初始位置
    # f_p = [-5,15]    # 1号车位置
    #     # vp_x, vp_y = formation_stagger(n_car, f_p)
    while(True):
        states = formation_ros.get_pose_states()
        if states == None:
            rospy.loginfo('waiting for gnss message')
            time.sleep(1)
            continue
        break
    vp_x = [states[i].position.x for i in range(n_car)]
    vp_y = [states[i].position.y for i in range(n_car)]
    # vp_x = [0,0,0]
    # vp_y = [0,-6.5,-3.8]
    # 队形：（三角队形）
    center_pos,theta1,r = [15,0],0,3
    target_x1,target_y1 = formation_triangle(center_pos,theta1,n_car,r)
    target_x1 = [x + origin_shift[0] for x in target_x1]
    target_y1 = [y + origin_shift[1] for y in target_y1]

    # 队形：(直线）
    tp_2,theta2,l_s=[30,0],90,3
    target_x2,target_y2 = formation_line(tp_2,theta2,n_car,l_s)
    target_x2 = [x + origin_shift[0] for x in target_x2]
    target_y2 = [y + origin_shift[1] for y in target_y2]
    
    # 障碍物坐标
    ob = np.array([
                   [5,2.5],
                   [5,-2.5],
                   [25,2.5],
                   [25,-2.5],
                   [25,5]
                   ])
    ob = ob + origin_shift + [1000, 1000]

    t1 =time.time()
    target_x1,target_y1=Assign(vp_x,vp_y,target_x1,target_y1)
    target_x2,target_y2=Assign(target_x1,target_y1,target_x2,target_y2)
    t2 = time.time()
    print(t2-t1)

    wp_x = [vp_x, target_x1, target_x2]
    wp_y = [vp_y, target_y1, target_y2]
    print("x坐标:",wp_x,'\ny坐标',wp_y)
    # wp_x, wp_y = extend_formation(wp_x, wp_y, 0)

    local_traj_gen(wp_x,wp_y,ob, formation_ros)


if __name__ == '__main__':
    main()
    # test_none_path()
