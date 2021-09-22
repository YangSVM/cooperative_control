#!/usr/bin/env python3

"""
Author: Shi Gongchen, Yang Yibin
在右手系中完成
生成全局路径，用frenet s坐标表示阶段。
参数队形保持阶段：(新：队形保持阶段轨迹生成函数：考虑如何保证多车位置速度相差可能较大的情况下，如何趋于同速)
队形变换阶段：(旧：匈牙利算法搭配规划函数得到。今天下午调试)
队形保持阶段


采样方式：
    - 横向终点位置采样(采样间距+车宽 是车能通过的最大间隙)
    - 终点时间采样：多久能够回到中心线
    - 终点纵向速度采样：达到终点后，车的速度是多少。
分别对横纵向用5次、4次多项式(关于时间t)，对起点状态和末端状态列方程，可以求解该轨迹。

"""

from typing import List
import rospy
import numpy as np
import copy
import math


from math import pi
import time
from formation_dec.formation_zoo import *
from scipy.optimize import linear_sum_assignment
from trajectory_tracking.msg import Trajectory, RoadPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from enum import Enum
from std_msgs.msg import Float64MultiArray

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise



# Parameter
MAX_SPEED = 50/3.6 # maximum speed [m/s]
MAX_ACCEL = 3.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1  # maximum curvature [1/m]

D_ROAD_W = 0.6  # road width sampling length [m]
ROAD_WIDTH_START = -1.2
ROAD_WIDTH_END = 1.2

DT = 0.2  # time tick [s]
# DT = 1
MAX_T = 4.0 # max prediction time [s]
MIN_T = 3.0  # min prediction time [s]
TARGET_SPEED = 1.5 # target speed [m/s]
D_T_S = TARGET_SPEED   # target speed sampling length [m/s]
# N_S_SAMPLE = 1  # sampling number of target speed
N_S_SAMPLE = 10  # sampling number of target speed

ROBOT_RADIUS = 1  # robot radius [m]
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
FormationState = Enum('FormationState', ('running', 'temp_goal', 'goal', 'waiting'))

class FormationROS():
    def __init__(self, n_car, car_ids:List):
        ''' 
        '''
        rospy.init_node("formation_core")
        self.n_car = n_car


        self.id_list = car_ids
        self.id_id_list_real = [i for i in range(n_car)]        # id_list中，第几辆车是真车，其余车是虚拟车。即id_list[1],id_list[2](2,5号车)是真车

        # 发布车辆轨迹
        self.pubs = [rospy.Publisher('car'+str(car_ids[i])+'/local_trajectory', Trajectory, queue_size=1) for i in range(n_car)]
        self.pub_csps = [rospy.Publisher('car'+str(car_ids[i])+'/global_trajectory', Trajectory, queue_size=1) for i in range(n_car)]
        self.pub_wp =rospy.Publisher('/temp_goal', Trajectory, queue_size=1)
        self.pub_task_time = rospy.Publisher('task_time_remain', Float64MultiArray, queue_size=1)
        # 接收所有车辆的信息
        for i in range(n_car):
            rospy.Subscriber('car'+str(car_ids[i])+'/gps', Odometry,  self.sub_gps_states, i)

        # 车辆状态
        self.pose_states = [Pose() for i in range(n_car)]
        self.gps_flag = [False for i in range(n_car)]

        self.rate = rospy.Rate(2)
        self.last_trajectory = None
        # 全局状态管理器：[起点，第一个队形点)为阶段0，[第一个队形点，第二个队形点)为阶段1，以此类推
        self.vehicle_formation_stage = [0 for i in range(n_car)]
        self.vehicle_states = [FormationState.running for i in range(n_car)]

    def sub_gps_states(self, msg, i):
        # rospy.loginfo('GNSS receive. Car index '+str(i))
        self.gps_flag[i] = True
        self.pose_states[i].position.x = msg.pose.pose.position.x
        self.pose_states[i].position.y = msg.pose.pose.position.y

        self.pose_states[i].orientation.z = msg.twist.twist.angular.z

    def get_pose_states(self):
        if all([self.gps_flag[x] for x in self.id_id_list_real] ) == True:
            return self.pose_states
        else:
            rospy.logwarn('gps info not ready!')
            print('gnss is received flag: ', self.gps_flag)
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
                rp.x = v_path_x[i][i_rp]
                rp.y = v_path_y[i][i_rp]
                rp.v = v_speed[i][i_rp]
                traj.roadpoints.append(rp)
            self.pubs[i].publish(traj)
    
    def pub_traj(self, trajs):
        for i in range(len(trajs)):
            self.pubs[i].publish(trajs[i])

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
    n_car  = wpx.shape[1]
    n_point = wpx.shape[0]
    traj = Trajectory()
    for i in range(n_point):
        
        for i_car in range(n_car):
            rp = RoadPoint()
            rp.x = wpx[i, i_car]
            rp.y = wpy[i, i_car]
            traj.roadpoints.append(rp)
    return traj

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


class QuarticPolynomial4LaneChange:

    def __init__(self, xs,  xe, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
 
        A = np.array([[3 * time ** 2, 2 * time, 1],
                      [6 * time, 2 , 0],
                      [time**3, time**2, time]])
        b = np.array([vxe,
                      axe,
                      xe- self.a0])
        x = np.linalg.solve(A, b)

        self.a1 = x[2]
        self.a2 = x[1]
        self.a3 = x[0]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t 

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 

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
    for di in np.arange(ROAD_WIDTH_START, ROAD_WIDTH_END, D_ROAD_W):

        # Lateral motion planning
        # 删除该采样
        for Ti in np.arange(MIN_T, MIN_T+1, 1):
            fp = FrenetPath()

            # lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuarticPolynomial4LaneChange(c_d,   di, 0.0, 0.0, Ti)
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
                # tfp.cd = K_J * Jp + K_T * Ti + K_D * sum(np.array(tfp.d[:] ))** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


# 给出参考轨迹csp, 求解对应frenet坐标系下的s-d
def cartesian2frenet(x, y, csp :cubic_spline_planner.Spline2D):
    pos = np.array([x, y])

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
    for i in range(len(ob)):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= (ROBOT_RADIUS/2) ** 2 for di in d])

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


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob, search_params):
    # t1 = time.time()
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    t2 = time.time()
    # print(t2-t1)
    fplist = calc_global_paths(fplist, csp)
    # t3 = time.time()

    if fplist is None:
        print('none fplist')

    fplist = check_paths(fplist, ob)
    if fplist is None or len(fplist) == 0:
        print('none fplist available')
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
# 先假设匀速运动: TODO 仍然存在逻辑问题，不知如何解决
def collision_avoid(local_trajs, pass_order):
    x, y, v=[],[],[]
    n_car = len(local_trajs)
    for i in range(n_car):
        x.append(np.array(local_trajs[i][0]))
        y.append(np.array(local_trajs[i][1]))
        v.append(np.array(local_trajs[i][2]))

    # 取x中最短的
    t_min = min([len(xi) for xi in x])

    ins_condition=[]  # 存储插入情况的列表，每个元素的形式为[j,ind_s,ind_e]

    is_collision_free = True
    for iter in range(10):  
        # 最多检查10个循环，直到冲突完全消解
        for ts in range(t_min):
            # 对每个时间点进行碰撞检测
            for i in range(n_car-1):
                for j in range(i+1, n_car):
                    # if v[i][ts] == 0 or v[j][ts] ==0:
                    #     break
                    d_ij = np.hypot(x[i][ts]-x[j][ts], y[i][ts]-y[j][ts])
                    if d_ij < ROBOT_RADIUS*2:
                        is_collision_free = False
                        # 在碰撞点前几个点停车
                        safe_ts = max(0, ts-5)
                        ins_condition.append([i, j, ts])
                        
                        # 根据通行次序确定停哪辆车
                        id_stop = j if pass_order[j] > pass_order[i] else i

                        v[id_stop][safe_ts:] = 0
                        x[id_stop][safe_ts:] = x[id_stop][safe_ts]
                        y[id_stop][safe_ts:] = y[id_stop][safe_ts]
                        rospy.logwarn('Collision avoidance! Car'+ str(j)+ ' stop at '+ str(0.1*ts) + '!' )
                        continue
        if is_collision_free:
            break

    # 碰撞点t_thre进行停车。速度置为0.x,y不变
    # for con
    # v[ts:] = 0
    return x,y,v


def Assign(pos_start, pos_end):
    # 匈牙利目标分配，并变换终点坐标以使编号保持一致
    x_start = pos_start[:, 0]
    y_start = pos_start[:, 1]
    x_end = pos_end[:, 0]
    y_end = pos_end[:, 1]
    n_car=len(x_start)
    cost_matrix = np.zeros((n_car, n_car))  # 代价矩阵
    for i in range(n_car):
        for j in range(n_car):
            # sq=(x_end[j]-x_start[i])**2+(y_end[j]-y_start[i])**2                # 代价为距离平方
            # sq=abs(x_end[j]-x_start[i])+abs(y_end[j]-y_start[i])                # 代价为曼哈顿距离
            sq=math.sqrt((x_end[j]-x_start[i])**2+(y_end[j]-y_start[i])**2)   # 代价为距离
            cost_matrix[i, j] = sq

    matches_i, matches_j = linear_sum_assignment(cost_matrix)

    # print("cost_matrix=")
    # print(cost_matrix)
    # print("assignment result=")
    # print(matches_i, matches_j)
    x_end_assign, y_end_assgin=[0]*len(x_end),[0]*len(y_end)

    # 变换编号。使得 pos_start 中的第i行的车，指派到 pos_end_assign 第i行的位置上
    for mi, mj in zip(matches_i, matches_j):
        x_end_assign[mi] = x_end[mj]
        y_end_assgin[mi] = y_end[mj]

    pos_end_assign = np.array([x_end_assign, y_end_assgin]).T

    return pos_end_assign, matches_i, matches_j


def cut_csp(csp:cubic_spline_planner.Spline2D, s0, delta_s=1.5):
    s_end = s0+delta_s
    s_max = csp.s[-1]
    n_points = int(delta_s /0.1)
    path_x, path_y, path_v = np.ones(n_points), np.ones(n_points), np.ones(n_points)*TARGET_SPEED
    for i in range(n_points):
        s = s0+i*0.1
        s = min(s, s_max-1e-3)
        path_x[i], path_y[i] = csp.calc_position(s)
        
    return path_x, path_y, path_v


def sequential_passing(pos_formations, states, global_frenet_csps, pos_formations_frenet,
                                                formation_ros: FormationROS,
                                                ):
    n_car = pos_formations.shape[1]
    # wp_x :  n_car*n_x
    path_x, path_y,path_v = [], [], []  # 各车依次路径点的x，y坐标及速度
    n_car = len(global_frenet_csps)
    csps = global_frenet_csps

    trajs = generate_traj_by_csp(csps)
    wp_traj = generate_traj_by_wp(pos_formations[:,:,0], pos_formations[:,:,1])
    for i_car in range(n_car):
        formation_ros.pub_csps[i_car].publish(trajs[i_car])
    formation_ros.pub_wp.publish(wp_traj)
    # 局部路径规划

    vehicle_states = formation_ros.vehicle_states
    vehicle_formation_stage = [0 for i in range(n_car)]         # 应该是每次都初始化
    last_trajectory = formation_ros.last_trajectory

    if states is None:
        # time.sleep(5)           # waiting for gnss message
        return
    # 1. 先决定通行次序.
    # 计算每辆车的stage
    # 计算所有车的frenet坐标
    frenet_car = np.ones([n_car, 2])
        
    for i_car in range(n_car):
        csp = global_frenet_csps[i_car]
        frenet_car[i_car, :] = cartesian2frenet(states[i_car].position.x, states[i_car].position.y, csp)
        for i_stage in range(pos_formations.shape[0]): 
            if frenet_car[i_car, 0] > pos_formations_frenet[i_stage, i_car, 0] - 1e-2:
                vehicle_formation_stage[i_car] = i_stage

    # 计算通行次序
    vehicle_formation_stage = np.array(vehicle_formation_stage)
    legal_cars = vehicle_formation_stage==0
    s_car2inter = frenet_car[:, 0] -  pos_formations_frenet[1, :, 0]
    s_car2inter[~legal_cars] = -1000
    car2pass = -1
    
    priority = np.argmax(s_car2inter)
    if s_car2inter[priority] <0:
        car2pass = priority



    # 阶段小于1的车， 按顺序启动。已经阶大于1的车，匀速前进
    for i_car in range(n_car):
        # 
        csp = global_frenet_csps[i_car]
        end_x = csp.sx.y[-1]
        end_y = csp.sy.y[-1]
        if np.hypot(states[i_car].position.x - end_x, states[i_car].position.y - end_y) <=1:
            print("Goal!")
            vehicle_states[i_car] = FormationState.goal
            path_x.append([])
            path_y.append([])
            path_v.append([])
            continue

        if vehicle_formation_stage[i_car]>0:
            tmp_x, tmp_y, tmp_v = cut_csp(global_frenet_csps[i_car], frenet_car[i_car, 0])
        else:
            if i_car == car2pass:
                tmp_x, tmp_y, tmp_v = cut_csp(global_frenet_csps[i_car], frenet_car[i_car, 0])
            else:
                tmp_x = np.ones(15) * states[i_car].position.x
                tmp_y = np.ones(15) * states[i_car].position.y
                tmp_v = np.ones(15) * states[i_car].orientation.z

        path_x.append(tmp_x)
        path_y.append(tmp_y)
        path_v.append(tmp_v)

    
    return path_x, path_y, path_v


def local_traj_gen(pos_formations, pass_orders, ob, states, 
                                        global_frenet_csp, 
                                        formation_ros: FormationROS, 
                                        pos_formations_frenet, 
                                        sample_basis=None,
                                        critical_state_selector: List =[]
                                        ):
    ''' 计算各车路径的函数，输入为全局路径需要经过的路点(每个路点为队形对应点)、障碍物位置
    params:
        pos_formations: np.array. [n_stage, n_car, 2]. 
        csp: 各车的全局路径
    '''

    # wp_x :  n_car*n_x
    path_x, path_y,path_v = [], [], []  # 各车依次路径点的x，y坐标及速度
    n_car = len(global_frenet_csp)
    csps = global_frenet_csp

    trajs = generate_traj_by_csp(csps)
    wp_traj = generate_traj_by_wp(pos_formations[:,:,0], pos_formations[:,:,1])
    for i_car in range(n_car):
        formation_ros.pub_csps[i_car].publish(trajs[i_car])
    formation_ros.pub_wp.publish(wp_traj)
    # 局部路径规划


    # 局部路径，用于存放v_x, v_y, v_s（横纵坐标速度）
    local_trajs_oral = [[[],[],[]] for i in range(n_car)]           # 单车算法规划的轨迹，未考虑协作碰撞
    paths = [[] for i in range(n_car)]                                          # 当前规划的轨迹

    vehicle_states = formation_ros.vehicle_states
    vehicle_formation_stage = formation_ros.vehicle_formation_stage
    last_trajectory = formation_ros.last_trajectory


    # 1. 首先规划出3条路径
    if states is None:
        # time.sleep(5)           # waiting for gnss message
        return

    for i_car in range(n_car):
        csp = csps[i_car]
        t_xy = time.time()
        s, d= cartesian2frenet(states[i_car].position.x, states[i_car].position.y, csp)
        t_xy2 = time.time()
        # print('转换时间：' , t_xy2 - t_xy)


        t_traj_start = time.time()
        # 判断是否到达队形点
        # 根据frenet坐标系s值判断
        for i_stage in range(pos_formations.shape[0]): 
            if s > pos_formations_frenet[i_stage, i_car, 0] - 1e-2:
                vehicle_formation_stage[i_car] = i_stage

        i_car_stage = vehicle_formation_stage[i_car] 
        print('car', i_car,': Stage', i_car_stage, '. Frenet sd:\t', (s, d))
        print('last state frenet sd: \t\t', pos_formations_frenet[i_car_stage, i_car, :])
        if i_car_stage+1<pos_formations.shape[0]:
            print('Next state frenet sd: \t\t', pos_formations_frenet[i_car_stage+1, i_car, :])
        else:
            # 道路末端没有预瞄点。不往前走。停住了
            print('goal reached!')
            vehicle_states[i_car] = FormationState.goal
            continue

        # 判断是否到达终点
        end_x = csp.sx.y[-1]
        end_y = csp.sy.y[-1]
        if np.hypot(states[i_car].position.x - end_x, states[i_car].position.y - end_y) <=1:
            print("Goal!")
            vehicle_states[i_car] = FormationState.goal
            
            continue

        isReady4Next = True
        # 如果三辆车都达到队形点，则进入过度期间继续下一阶段队形变换，否则停车等待
        for other_car in range(n_car):
            if (other_car == i_car) or (other_car not in formation_ros.id_id_list_real):
                # 若为自车或者虚拟车，不判断
                continue
            if vehicle_formation_stage[i_car] >  vehicle_formation_stage[other_car]:
                # 如果自车领先他车一个阶段，自车需要先等待.
                isReady4Next  = False
        t_traj_end = time.time()
        # print('阶段检测时间：', t_traj_end- t_traj_start)

        if isReady4Next:
            # rospy.loginfo('is ready for next transition')
            vehicle_states = [FormationState.running for i in range(n_car)]
        else:
            print('car', i_car,': temp goal. waiting')
            if last_trajectory is None or len(last_trajectory[0][i_car]) ==0:
                rospy.logwarn('car' + str(i_car) +' last trajectory is None')
                continue
            path_x, path_y, path_v = last_trajectory
            tmp_v = path_v[i_car][:]
            # 速度直接置为零
            path_v[i_car][:]= np.zeros(tmp_v.shape)
            path_x[i_car][:] = np.ones(path_x[i_car].shape) * (states[i_car].position.x + 1e-3)
            path_y[i_car][:] = np.ones(path_y[i_car].shape) * (states[i_car].position.y + 1e-3)
            # x,y也需要裁剪
            # safe_i = min(len(path_x[i_car])-1, 5)
            # path_x[i_car][safe_i:] = np.ones(path_x[i_car][safe_i:].shape)*path_x[i_car][safe_i]
            # path_y[i_car][safe_i:] = np.ones(path_y[i_car][safe_i:].shape)*path_y[i_car][safe_i]
            local_trajs_oral[i_car][0] = path_x[i_car][:]
            local_trajs_oral[i_car][1] = path_y[i_car][:]
            local_trajs_oral[i_car][2] = path_v[i_car][:]
            continue
        
        # rospy.loginfo('car'+str(i_car) +' formation stage:' + str( vehicle_formation_stage[i_car]))
        
        s_d = TARGET_SPEED
        # s, d= xy2frenet(states[i_car].position.x, states[i_car].position.y, csp)
        t_traj_start = time.time()
        path = frenet_optimal_planning(csp, s, s_d, d, 0, 0, ob, 0)
        if path is None:
            assert last_trajectory is not None
            path = FrenetPath()
            path.x = last_trajectory[0][i_car]
            path.y = last_trajectory[1][i_car]
            path.s_d = last_trajectory[2][i_car]
            rospy.logwarn('Car'+str(i_car)+' planning return None. Used last trajectory')

        t_traj_end = time.time()
        # print('轨迹规划时间：', t_traj_end- t_traj_start)

        if path is None:
            print('cannot plannning trajectory')
            break
        # 存储单车的路径
        local_trajs_oral[i_car][0] = path.x
        local_trajs_oral[i_car][1] = path.y
        local_trajs_oral[i_car][2] = path.s_d
        paths[i_car] = path

    # 2. 速度规划。考虑车辆之间避撞
    t_traj_start = time.time()
    path_x, path_y, path_v = collision_avoid(local_trajs_oral, pass_orders[min(vehicle_formation_stage)])
    t_traj_end = time.time()
    # print('避撞计算时间：', t_traj_end- t_traj_start)

    # 3. 将信息保存到全局中。下一次规划继续使用
    formation_ros.last_trajectory = path_x, path_y, path_v
    formation_ros.vehicle_formation_stage =vehicle_formation_stage
    formation_ros.vehicle_states = vehicle_states
    return path_x, path_y, path_v


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
    s_d = 1.5
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
    n_car=2

    # ros init
    formation_ros = FormationROS(n_car)
  
    # 坐标原点偏移量
    origin_shift = np.array([-85, -4])

    # 直线全局参考线
    global_x = [0,50]
    global_y = [0,0]
    global_frenet_csp = cubic_spline_planner.Spline2D(global_x, global_y)

    # 队形1。车辆初始位置
    while(True):
        states = formation_ros.get_pose_states()
        if states == None:
            rospy.loginfo('waiting for gnss message')
            time.sleep(1)
            continue
        break

    pos_init = np.empty([n_car, 2])
    for i in range(n_car):
        pos_init[i, 0] = states[i].position.x
        pos_init[i, 1] = states[i].position.y

    # 队形2：纵向直线。
    tp_2,theta2,d_car=[10,0],90,3
    pos_formation1 = formation_line(tp_2,theta2,n_car,d_car)
    pos_formation1 = pos_formation1 + origin_shift
    
    # non-obstacle
    ob = []

    t1 =time.time()
    pos_formation1 = Assign(pos_init, pos_formation1)

    t2 = time.time()
    print(t2-t1)

    pos_formations = np.array([pos_init, pos_formation1])
    print("初始位置：\n", pos_init, "\n结束位置：\n", pos_formation1)


    local_traj_gen(pos_formations, ob, formation_ros, global_frenet_csp)


if __name__ == '__main__':
    main()
    # test_none_path()
