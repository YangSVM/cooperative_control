#!/usr/bin/env python3

"""
v 1.0
新版编队控制
"""
from enum import Enum
from typing import Dict, List, Tuple
from formation.scripts.cubic_spline_planner import Spline2D
import time

from formation_common.cubic_spline_planner import cartesian2frenet, spline_expand
# from formation_common.config_formation_continous import *

from formation_common.formation_zoo import *
from formation_continous.src.load_task_points import search
from ros_interface import  ROSInterface, MATrajPlanningBase

from formation_common.global_planning import generate_route, ds_trans_init
from formation_common.formation_core import assgin_traj, judge_formation_end, judge_formation_stage, Assign
from trajectory_planning import trajectory_planning, FrenetPath
from geometry_msgs.msg import Pose
from formation_common.config_formation_continous import TARGET_SPEED
from scipy.spatial.distance import cdist

# 编队状态：稳态。避障。数目增减（出队、入队）。队形变换。
class FormationState(Enum):
    Stable = 1
    Obstacle=2
    Number = 3
    Transition=4

DV_RATIO = 0.6

# formation state machine
class FormationSM():
    def __init__(self, init_state=FormationState.Stable) :
        self.state = init_state

    def state_transition(self, stage, obs)->FormationState:
        '''汽研所场景下，简化版本的状态判定。根据头车位置，以及场景中是否有障碍物进行区分。
        '''
        if obs is not None and len(obs) !=0:
            self.state = FormationState.Obstacle
        else:
            # if stage==-3:
            #     self.state = FormationState.Stable      # 用平行策略控制
            if stage<0:
                self.state = FormationState.Transition
            else:
                self.state = FormationState.Stable

        return self.state

# 编队轨迹规划
class FormationTrajPlanning(MATrajPlanningBase):
    def __init__(self, n_car, center_line, formation_begin_s, formation_types, d_car=2.5,ds_trans=[], individual_csps:List[Spline2D]=[]) -> None:
        '''
        Params:
            theta_degs: 直接控制individual队形中的角度，而不是采用全局轨迹中的角度生成队形
        '''
        # super().__init__(n_car, car_ids)
        self.n_car = n_car
        self.d_car = d_car
        self.fsm = FormationSM()
        self.formation_begin_s = formation_begin_s
        self.ds_trans = ds_trans
        self.formation_types = formation_types
        self.formation_type_t = FormationState.Stable         # 当前编队类型
        self.stage = 0              # 当前所在的队形状态
        # if center_line.shape[0] ==14:
        #     print('debug')

        # 如果有各自轨迹，直接使用
        if len(individual_csps) > 0:
            self.individual_csps = individual_csps
        else:
            # 根据全局轨迹，以及编队队形变换要求，生成预设轨迹
            self.global_csp,  self.individual_csps, self.matches, self.formation_poses  = \
                generate_route(center_line, formation_begin_s, formation_types, n_car, d_car=d_car,ds_trans=ds_trans)
        
        # if center_line.shape[0] ==14:
        plt.plot(center_line[:,0], center_line[:,1],'g*-')

        for i, csp_i in enumerate(self.individual_csps):
            x,y, _ ,_,_= spline_expand(csp_i)
            plt.plot(x,y, 'r*-')
        plt.axis('equal')

        self.leader_id = -1
        self.isInit = False         # 是否已经初始化。
        self.isSeqPassingInit = False
        self.cp_seq_pass = np.array([0,0]) 
        self.passing_order = [-1 for i in range(self.n_car)]

    def initial(self, pose_states: List[Pose]):
        '''给每辆车分配一条 最近的 轨迹
        Params:
            pose_states: len=n_car.存储每辆车的位置
        '''
        # 直接按照初始位置分：不鲁棒。车辆可能靠前导致失败
        csp_poses = np.array([[csp.calc_position(0)] for csp in self.individual_csps])
        csp_poses = csp_poses.reshape([-1,2])
        car_poses = np.array([[pose.position.x, pose.position.y] for pose in pose_states])
        car_poses = car_poses.reshape([-1,2])
        # _,_,matches=Assign(csp_poses, car_poses)
        matches = assgin_traj(car_poses, self.individual_csps)
        self.individual_csps =[self.individual_csps[i] for i in matches]            # reorder。根据分配结果重新排列

        # 记录头车编号：中间的轨迹的车。偶数量则是中间靠后(4//2 =2，第三辆车)。
        mid_id = self.n_car//2 
        self.leader_id = np.where(matches==mid_id)[0][0]

        # ds_trans 初始化
        delta_s_end = 1         # 当处于这个阶段内。
        self.ds_trans = ds_trans_init(self.ds_trans, len(self.formation_begin_s), self.formation_begin_s, self.individual_csps[self.leader_id].s[-1] - delta_s_end)

    
    def traj_planning(self, pose_states, obs=[])->Tuple[List[np.ndarray], float]:
        '''
        Params:
            pose_states: 
        Return:
            trajs: length n_car。存储每辆车的轨迹x,y
            t_remain: 剩余时间。-1表示
        '''
        if not self.isInit:
            self.isInit = True
            self.initial(pose_states)
        # 各车辆状态: [n_car, 3]: x,y, yaw(弧度制)。SI
        poses = [[pose.position.x, pose.position.y, pose.orientation.z] for pose in pose_states]
        poses = np.array(poses).reshape(-1,3)
        li = self.leader_id

        s_leader, _ = cartesian2frenet(poses[li, 0], poses[li, 1], self.individual_csps[li])
        stage = judge_formation_stage([s_leader], self.formation_begin_s, self.ds_trans)[0]
        
        self.stage = stage
     
        print('\n'+10*'*' + 'formation stage: ', stage, '.\t formation type: ', self.formation_types[int(abs(stage))])
        print(10*'*' + 's_leader: ', s_leader)

        print('car poses:\n ', poses)
        state = self.fsm.state_transition(stage, obs)
        self.formation_type_t = self.formation_types[int(abs(stage))]         # 当前编队状态

        # 单车规划各自路径
        fps = []
        isGoals =[]
        for i_car in range(self.n_car):
            fp, isGoal = trajectory_planning(self.individual_csps[i_car], poses[i_car])
            fps.append(fp)
            isGoals.append(isGoal)

        # 速度规划
        if state == FormationState.Stable:
            fps = self.stable_control(fps, poses, s_leader)
        elif state == FormationState.Obstacle:
            fps = self.sequential_passing(self.individual_csps, poses)
        elif state == FormationState.Transition:
            fps = self.transition_control(fps, poses, s_leader)

        for fp, isGoal in zip(fps, isGoals):
            if isGoal:
                print('is goal',20*'-*')
                fp.set_v(0)

        trajs = [fp.to_numpy() for fp in fps]

        isDone = judge_formation_end(poses[:, :2], self.individual_csps)
        if isDone:
            t_remain = -1
        else:
            t_remain = (self.individual_csps[self.leader_id].s[-1] - s_leader) / TARGET_SPEED
            if t_remain<0:          # 有可能头车到达，但是其他车没到达。
                s_cars = [ cartesian2frenet(poses[li, 0], poses[li, 1], self.individual_csps[li])[0] for li in range(self.n_car)]
                t_remains = [ (self.individual_csps[i].s[-1] - s_cars[i]) / TARGET_SPEED for i in range(self.n_car)]
                t_remain = max(t_remains)

        return trajs, t_remain


    def stable_control(self, fps:List[FrenetPath], poses, s_leader):
        ''' 稳态队形控制。根据每辆车的位置，控制速度。

        '''
        # 根据头车位置生成队形
        # x,y = self.individual_csps[self.leader_id].calc_position(s_leader)
        pos_leader = poses[self.leader_id, :]
        yaw = self.individual_csps[self.leader_id].calc_yaw(s_leader)

        formation_np = gen_formation(pos_leader, yaw, self.n_car, self.d_car, self.formation_type_t)
        formation_np, _, _ = Assign(poses, formation_np)

        # 比例控制
        yaw_vec = np.array([math.cos(yaw), math.sin(yaw)])
        delta_vec = formation_np - poses[..., :2]
        delta_d = np.linalg.norm(delta_vec, axis=1)
        direction = (yaw_vec * delta_vec).sum(axis=1)            # 点积判断速度方向
        dt = 5            # 下一次轨迹规划完成时，能够调整好队形
        d_v  =  delta_d* np.sign(direction) / dt
        # 20% 以内的加减速
        d_v[d_v>DV_RATIO*TARGET_SPEED] = DV_RATIO*TARGET_SPEED
        d_v[d_v<-DV_RATIO*TARGET_SPEED] = -DV_RATIO*TARGET_SPEED
        v = TARGET_SPEED +d_v

        v_str = ['{:.2f}'.format(vi) for vi in v]
        print('!!![Stable]: v: ', v_str)

        for i in range(self.n_car):
            fps[i].set_v(v[i])

        return fps

    def transition_control(self, fps, poses, s_leader):
        # 目标队形生成
        i_stage = int(abs(self.stage))
        s_target = self.formation_begin_s[i_stage] + self.ds_trans[i_stage]

        pos_leader = poses[self.leader_id, :]
        pos_leader =self.individual_csps[self.leader_id].calc_position(s_target)
        yaw = self.individual_csps[self.leader_id].calc_yaw(s_target)

        formation_np = gen_formation(pos_leader, yaw, self.n_car, self.d_car, self.formation_type_t)
        # formation_np, _, _ = Assign(poses, formation_np)                        # 这样写会导致编号动态变化。
        formation_np = formation_np[self.matches[i_stage, :], :]
        

        yaw_vec = np.array([math.cos(yaw), math.sin(yaw)])

        # 计算路程
        s_formation = [cartesian2frenet(pos[0],pos[1],self.individual_csps[i_car])[0] for i_car, pos in enumerate(formation_np)]
        s_pos = [cartesian2frenet(pos[0],pos[1],self.individual_csps[i_car])[0] for i_car, pos in enumerate(poses)]
        delta_s = [s1-s2 for s1,s2 in zip(s_formation, s_pos)]
        s0 = delta_s[self.leader_id]
        delta_s = np.array([s - s0  for s in delta_s])
        print('detla_s', delta_s)

        # 计算直接距离
        delta_vec = formation_np - poses[..., :2]
        delta_d_norm = np.linalg.norm(delta_vec, axis=1)
        direction = (yaw_vec * delta_vec).sum(axis=1)            # 点积判断速度方向


        # 前馈控制 - 根据距离计算之后的速度
        dt = 3            # 下一次轨迹规划完成时，能够调整好队形
        # d_v  =  delta_d_norm* np.sign(direction) / dt

        d_v  =delta_s/dt
 
        # DV_RATIO 以内的加减速
        d_v[d_v>DV_RATIO*TARGET_SPEED] = DV_RATIO*TARGET_SPEED
        d_v[d_v<-DV_RATIO*TARGET_SPEED] = -DV_RATIO*TARGET_SPEED
        v = TARGET_SPEED +d_v

        v_str = ['{:.2f}'.format(vi) for vi in v]
        print('!!![Transition]: v: ', v_str)

        # 设置成同时能够到达目标位置的速度
        for i in range(self.n_car):
            fps[i].set_v(v[i])

        # 检查是否会发生碰撞。如果碰撞，按照给定规则停车。
        # fps = collision_avoid(fps, v, pass_order=[self.n_car - i for i in range(self.n_car)])


        return fps
    
    def sequential_passing_init(self, csps:List[Spline2D], car_poses:np.ndarray):
        t1 = time.time()
        # 识别冲突点：(现在默认只有一个冲突点，并且所有轨迹总共一个冲突点)
        if len(csps)<3:
            print('sequantial passing not enough points.')
            return None
        
        roadpoints = []
        for i in range(2):
            csp = csps[i]
            rx, ry, _, _, _ = spline_expand(csp, ds=0.1)
            pose = np.array([rx, ry]).T
            roadpoints.append(pose)
        
        # 找距离接近的点中，最早的重合点
        distance  = cdist(roadpoints[0], roadpoints[1], metric='euclidean')
        idx, _ = np.where(distance< 0.1)
        ix = np.min(idx)

        t2 = time.time()
        print(t2-t1)
        cp = roadpoints[0][ix, :]
        
        # pose frenet 计算
        pos_formations_frenet = np.zeros([2, self.n_car, 1])
        for i, csp in enumerate(csps):
            pos_formations_frenet[1, i, 0], _ = cartesian2frenet(cp[0], cp[1], csp)

        # 通行次序为从右往左
        passing_order = np.argsort(-car_poses[:, 0])

        return cp, pos_formations_frenet, passing_order

    # def sequential_passing(pos_formations, states, global_frenet_csps, pos_formations_frenet):
    def sequential_passing(self, csps: List[Spline2D], poses: np.ndarray):
        ''' 顺序通行
        '''
        if self.isSeqPassingInit == False:
            self.isSeqPassingInit = True
            cp, pos_formations_frenet, passing_order = self.sequential_passing_init(csps, poses)     # collision point
            self.cp_seq_pass =cp
            self.pos_formations_frenet_seq_pass = pos_formations_frenet
            self.passing_order  =passing_order
        else:
            cp = self.cp_seq_pass
            pos_formations_frenet = self.pos_formations_frenet_seq_pass
            passing_order = self.passing_order
        n_car = self.n_car

        
        # wp_x :  n_car*n_x
        path_x, path_y,path_v = [], [], []  # 各车依次路径点的x，y坐标及速度

        # 局部路径规划

        # vehicle_states = formation_ros.vehicle_states
        vehicle_formation_stage = [0 for i in range(n_car)]         # 车辆与关键点的相对位置
        # last_trajectory = formation_ros.last_trajectory

        # 1. 先决定通行次序.
        # 计算每辆车的stage。 stage>0的车，正常运行。stage==0的车，只能走一辆。
        # 计算所有车的frenet坐标
        frenet_car = np.ones([n_car, 2])
            
        for i_car in range(n_car):
            csp = csps[i_car]
            frenet_car[i_car, :] = cartesian2frenet(poses[i_car, 0], poses[i_car, 1], csp)
            for i_stage in range(pos_formations_frenet.shape[0]): 
                if frenet_car[i_car, 0] > pos_formations_frenet[i_stage, i_car, 0] - 1e-2:
                    vehicle_formation_stage[i_car] = i_stage

        # 筛选合法的车
        vehicle_formation_stage = np.array(vehicle_formation_stage)
        legal_cars = vehicle_formation_stage==0

        # 计算通行次序: 离的远的先出发
        # s_car2inter = frenet_car[:, 0] -  pos_formations_frenet[1, :, 0]            # 各个车与交叉点的距离
        # s_car2inter[~legal_cars] = -1000            # -1000<所有stage 0的车辆的距离
        # car2pass = -1                   # 将要通行的车
        
        # priority = np.argmax(s_car2inter)
        # if s_car2inter[priority] <0:
        #     car2pass = priority

        # 计算通行次序：从右往左开        
        passing_order = passing_order[legal_cars]
        if len(passing_order)>0:
            car2pass = passing_order[0]
            print('seq pass: car ', car2pass, 'starting! ')
        else:
            car2pass = -1         # 所有车都已经启动完成


        # 车辆轨迹控制
        # 阶段小于1的车， 按顺序启动。已经阶大于1的车，匀速前进
        for i_car in range(n_car):
            # 
            csp = csps[i_car]
            end_x = csp.sx.y[-1]
            end_y = csp.sy.y[-1]
            if np.hypot(poses[i_car, 0] - end_x, poses[i_car, 1]- end_y) <=1:
                print("Goal!")
                path_x.append([])
                path_y.append([])
                path_v.append([])
                continue

            if vehicle_formation_stage[i_car]>0:            # 已经通过的车辆正常通行
                tmp_x, tmp_y, tmp_v = cut_csp(csps[i_car], frenet_car[i_car, 0])
            else:           # car2pass
                if i_car == car2pass:
                    tmp_x, tmp_y, tmp_v = cut_csp(csps[i_car], frenet_car[i_car, 0])
                else:
                    tmp_x = np.ones(15) * poses[i_car, 0]
                    tmp_y = np.ones(15) * poses[i_car, 1]
                    tmp_v = np.ones(15) * 0

            path_x.append(tmp_x)
            path_y.append(tmp_y)
            path_v.append(tmp_v)

        fps = [FrenetPath() for i in range(n_car)]
        for i, fp in enumerate(fps):
            fp.x = path_x[i]
            fp.y = path_y[i]
            fp.v = path_v[i]

        return fps


def cut_csp(csp:Spline2D, s0, delta_s=10):
    s_end = s0+delta_s
    s_max = csp.s[-1]
    n_points = int(delta_s /0.1)
    path_x, path_y, path_v = np.ones(n_points), np.ones(n_points), np.ones(n_points)*TARGET_SPEED
    for i in range(n_points):
        s = s0+i*0.1
        s = min(s, s_max-1e-3)
        path_x[i], path_y[i] = csp.calc_position(s)
        
    return path_x, path_y, path_v
    

# 碰撞检测：按照给定速度，离散化点列，查看是否
# 修改成分离轴 或 两个小圆
def collision_avoid(fps:List[FrenetPath],  v_fps, pass_order):
    ''' 先把每条轨迹进行样条曲线插值。假设横向控制完美。按照质点检测碰撞。
    根据速度每 0.1s 生成下一时刻s,根据样条曲线反算出 x,y
    '''
    ROBOT_RADIUS = 1.0

    T = 4   # 预测未来T秒的时长
    dt = 0.1    # 预测步长为dt
    t = np.arange(0,T,dt)

    # 所有车辆的位置. list [ np.ndarray]    
    n_car = len(fps)
    pos = [np.zeros([len(t), 2]) for i in range(n_car)]

    # csps = [Spline2D(fp.x, fp.y) for fp in fps]
    csps =[]
    for i, fp in enumerate(fps):
        if len(fp.x) < 2:
            # 此时代表已经有到达终点的轨迹
            pos[i] = np.ones([len(t), 2]) 
            pos[i][:, 0] = fp.x
            pos[i][:, 1] = fp.y
            csps.append(None)
        else:
            csps.append(Spline2D(fp.x, fp.y))



    for i in range(n_car):
        if csps[i] is None:
            continue
        s = v_fps[i]* t
        for ti, si in enumerate(s):
            pos[i][ti, 0], pos[i][ti, 1] = csps[i].calc_position(si)

        pos[i] = pos[i][~(np.isnan(pos[i][:, 0])), :]
        pos[i] = pos[i][~(np.isnan(pos[i][:, 1])), :]
        # pos[i] = 

    x, y, v=[],[],[]
    n_car = len(fps)
    for i in range(n_car):
        x.append(np.array(pos[i][ :, 0]))
        y.append(np.array(pos[i][ :, 1]))
        v.append(v_fps[i]* np.ones(t.shape))

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
                        print('[warning!] Collision avoidance! Car'+ str(j)+ ' stop at '+ str(0.1*ts) + '!' )
                        continue
        if is_collision_free:
            break

    # 碰撞点t_thre进行停车。速度置为0.x,y不变
    # for con
    # v[ts:] = 0

    for i in range(n_car):
        fps[i].x = x[i]
        fps[i].y = y[i]
        fps[i].v = v[i]

    return fps


def collision_avoid2(fps:List[FrenetPath],  v_fps, pass_order):
    ''' 根据轨迹情况(v(s))计算s(t), yaw(t)，离散化时间，根据车辆几何外形，判断是否碰撞
    '''
    pass


class MAFormationTrajPlanning(MATrajPlanningBase):
    ''' color MAPF. 同质智能体可以交换目的地。
    '''
    def __init__(self,  car_colors, center_lines, formation_begin_s, formation_types, d_car=2.5, ds_trans=[]) -> None:
        self.center_lines = center_lines

        self.n_groups = len(center_lines)

        self.n_group_cars = np.array([len(car_color) for car_color in car_colors])
        self.color_group_dict = {}
        for i, colors in enumerate(car_colors):
            color = colors[0]       # 假定同组里面颜色相同
            if color in self.color_group_dict.keys():
                self.color_group_dict[color].append(i)
            else:
                self.color_group_dict[color] = [i]
        self.colors = list(self.color_group_dict.keys())                        # 所有group中的color
        self.n_color = len(self.colors)                                                         # 所有color的数量

        # 计算同组的智能体的id编号. id编号原则。先编排小序号的(pix<极创<jjuv)
        self.color_car_dict = {}
        self.car_color_list = []
        for car_color in car_colors:
            self.car_color_list.extend(car_color)
        self.car_color_list = np.array(self.car_color_list)
        
        self.car_color_list = np.sort(self.car_color_list)      #排序。相当于先排小的

        for color in self.colors:
            self.color_car_dict[color] = np.where(self.car_color_list == color)[0]
        
        color_n_car_dict = {}
        for color, color_car_ids in self.color_car_dict.items():
            color_n_car_dict[color] = len(color_car_ids)
        self.color_start=0

        self.car_colors = car_colors
        self.ftps = []

        self.individual_csps=[]
        for i_group in range(self.n_groups):
            ftp = FormationTrajPlanning(len(car_colors[i_group]), center_lines[i_group], formation_begin_s[i_group], formation_types[i_group], d_car=d_car, ds_trans=ds_trans[i_group])
            self.ftps.append(ftp)
            self.individual_csps.extend(ftp.individual_csps)
        self.isInit = False

    def initial(self, poses):
        ''' 按照位置预先分配一次轨迹。
        Return:
            self.group_car_dict: 每个group里面对应的车辆ID
        '''
        self.isInit = True
        poses = np.array([[pose.position.x, pose.position.y, pose.orientation.z] for pose in poses])

        # 计算结果：每个group对应的车辆ID
        self.group_car_dict = {}

        for color in self.colors:
            group_ids = self.color_group_dict[color]

            n_group_car  = self.n_group_cars[np.array(group_ids)]
            group_start_id = np.cumsum(n_group_car)
            group_start_id = np.insert(group_start_id, 0, 0)

            csp_pos = -1 * np.ones([sum(n_group_car), 2])

            for i, group_id in enumerate(group_ids):
                csp_pos[group_start_id[i]: group_start_id[i]+n_group_car[i], :] = self.center_lines[group_id][0, :2]
            car_pos = poses[np.array(self.color_car_dict[color]), :]

            _,  matches = Assign(csp_pos, car_pos)
            for i, group_id in enumerate(group_ids):
                gsi = group_start_id[i]     # group start index
                gei = group_start_id[i]+n_group_car[i]      # group end index
                self.group_car_dict[group_id] = self.color_car_dict[color][matches[gsi:gei]]

                self.group_car_dict[group_id] = np.sort(self.group_car_dict[group_id])

        return 0

    def traj_planning(self, poses, obs=[]):
        ''' 多群体轨迹各自规划
        '''
        if not self.isInit:
            self.initial(poses) 
        trajs_group = []
        car_ids = []
        t_remains = []
        # 按照group_car_dict 进行组装并发送
        for i_group in range(self.n_groups):
            poses_i_group = [poses[i_car] for i_car in self.group_car_dict[i_group]]
            trajs, t_remain = self.ftps[i_group].traj_planning(poses_i_group, obs)
            trajs_group.extend(trajs)
            t_remains.append(t_remain)
            car_ids.extend(self.group_car_dict[i_group])
        
        trajs_group_new = [0  for i_car in car_ids]
        for i, i_car in enumerate(car_ids):
            trajs_group_new[i_car] = trajs_group[i]

        t_remain = max(t_remains)
        return trajs_group_new, t_remain
        

def main4ros(n_car, car_ids, center_lines, formation_begin_ls, formation_type_ls,  ds_trans_ls):

    # ftp = FormationTrajPlanning(n_car, car_ids, center_line, formation_begin_s, formation_types, ds_trans=ds_trans,d_car=2.5)
    # ftp = MAFormationTrajPlanning(n_car, car_ids, center_lines[1], formation_begin_ls[1], formation_type_ls[1],  ds_trans=ds_trans_ls[1],d_car=2.5)
    ftp = MAFormationTrajPlanning([[0,0],[0],[0,0]],  center_lines, formation_begin_ls, formation_type_ls,  ds_trans=ds_trans_ls, d_car=2.5)

    unique_car_ids = [i+1 for i in range(n_car)]
    ri = ROSInterface(n_car, unique_car_ids)
    ri.initial()
    ri.running(ftp)

    return 0


if __name__ == '__main__':
    from formation_common.config_formation_continous import *
    from load_task_points import search

    car_colors, center_lines, formation_begin_ls, formation_type_ls,  ds_trans_ls = search()
    main4ros(n_car, car_colors, center_lines, formation_begin_ls, formation_type_ls,  ds_trans_ls)
