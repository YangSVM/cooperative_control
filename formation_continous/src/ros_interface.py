#!/usr/bin/env python3

"""
v 1.0
处理与ros之间的消息通信

"""
from abc import abstractclassmethod
from math import pi
import matplotlib.pyplot as plt
from formation_common.cubic_spline_planner import spline_expand

import rospy
from trajectory_tracking.msg import Trajectory, RoadPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import time
from typing import List, Tuple

from communication.msg import ExecInfoRequest, TasksRequest, AttackRequest, TaskEI, Task

from enum import Enum

class MATrajPlanningBase():
    ''' 轨迹接口类。必须具备 traj_planning 虚函数。
    '''
    def __init__(self) -> None:
        pass

    @abstractclassmethod
    def traj_planning(self, poses_states, obs=[])->Tuple[List[np.ndarray], float]:
        ''' 虚函数。子函数必须给出轨迹规划函数
        Params:
            self:
                全局轨迹        

            poses_states    车辆坐标      
            obs: 障碍物位置。
        Return:
            trajs: 各个车之后轨迹。Lists of numpy(n*3), 列表长度为n(车辆数目)。[x,y,v]
            
            t_remain: 预计多久之后完成。-1表示已完成
        '''
        pass


# ros接口：完成某一具体任务时的规划。
class ROSInterface():
    def __init__(self, n_car, car_ids, id_id_list_real=[]) -> None:
        rospy.init_node("ros_interface_for_formatinon_planning")
        self.n_car = n_car
        # 调试的是第几辆车
        self.car_ids = car_ids
    
        if len(id_id_list_real) ==0:
            self.id_id_list_real = [i for i in range(n_car)]        # id_list中，第几辆车是真车，其余车是虚拟车。即car_ids[1],car_ids[2](2,5号车)是真车
        else:
            self.id_id_list_real = id_id_list_real
        # 发布车辆轨迹
        self.__pubs = [rospy.Publisher('car'+str(car_ids[i])+'/local_trajectory', Trajectory, queue_size=1) for i in range(n_car)]
        # 辅助使用的全局轨迹和关键点的信息
        self.__pub_csps = [rospy.Publisher('car'+str(car_ids[i])+'/global_trajectory', Trajectory, queue_size=1) for i in range(n_car)]
        self.__pub_wp =rospy.Publisher('/traj_key_point', Trajectory, queue_size=1)
        # 接收所有车辆的信息
        for i in range(n_car):
            rospy.Subscriber('car'+str(car_ids[i])+'/gps', Odometry,  self.sub_gps_states, i)

        # 车辆状态
        self.pose_states = [Pose() for i in range(n_car)]               # 国际单位制。弧度
        self.gps_flag = [False for i in range(n_car)]                     # 判断每辆车是否返回GNSS信息

        self.rate = rospy.Rate(1)

    def sub_gps_states(self, msg, i):
        self.gps_flag[i] = True
        self.pose_states[i].position.x = msg.pose.pose.position.x
        self.pose_states[i].position.y = msg.pose.pose.position.y
        # 航向角用z轴转向表示。弧度制
        self.pose_states[i].orientation.z = msg.twist.twist.angular.z/180*pi

    def get_pose_states(self):
        if all([self.gps_flag[x] for x in self.id_id_list_real] ) == True:
            return self.pose_states
        else:
            rospy.logwarn('gps info not ready!')
            print(self.gps_flag)
            return None


    
    def initial(self):
        '''初始化。保证接收到所有车辆位置。
        '''
        while not rospy.is_shutdown():
            poses = self.get_pose_states()
            
            if poses is not None:
                # 得到所有位置后退出初始化
                break
            self.rate.sleep()
        
    def running(self, ftp:MATrajPlanningBase):
        
        while not rospy.is_shutdown():


            t_begin = time.time()
            trajs, t_remain = ftp.traj_planning(self.pose_states)

            if t_remain < 0 :
                print('Formation is Done')
                break

            t_end = time.time()
            rospy.loginfo("Planning time: "+str(t_end - t_begin))
            # plt.clf()
            
            # for i_traj in range(self.n_car):
            #     plt.plot(trajs[i_traj][:, 0], trajs[i_traj][:, 1], 'r*-')
            # plt.axis('equal')
            
            self.publish_trajs(trajs)

            # 辅助性画图工具
            self.publish_csps(ftp.individual_csps)
            self.rate.sleep()

    def publish_trajs(self, trajs):
        for i, traj in enumerate(trajs):
            traj_ros = traj_np2ros_msg(traj)
            self.__pubs[i].publish(traj_ros)


    def publish_csps(self, csps):
        '''
        '''
        n_car = self.n_car
        for i_car in range(len(csps)):
            # 先转成np, 然后转成traj 发布出去
            x,y,_,_,_= spline_expand(csps[i_car], ds=0.1)
            n = len(x)
            traj_np = np.ones([n, 3])
            traj_np[:, 0] = x
            traj_np[:, 1] = y
            traj_ros = traj_np2ros_msg(traj_np)
            self.__pub_csps[i_car].publish(traj_ros)

    def publish_wp(self, trajs):
        self.__pub_wp(trajs)


def traj_np2ros_msg(traj_np):
    n_point = traj_np.shape[0]
    traj = Trajectory()

    for i in range(n_point):
        rp = RoadPoint()
        rp.x = traj_np[i, 0]
        rp.y = traj_np[i, 1]
        rp.v = traj_np[i, 2]
        traj.roadpoints.append(rp)
    return traj


class TaskType(Enum):
    BuildUp = 1             # 集结
    Search=2                    # 侦查
    Battle = 3                  # 打击


# 任务级的接口：任务间的转换。任务完成信息的反馈。任务的启动和加载。
class TaskManager():
    def __init__(self) -> None:        

        rospy.Subscriber('TasksRequest', TasksRequest, self.task_request_callback)
        rospy.Subscriber('AttackRequest', AttackRequest, self.attack_request_callback)
        # time step for start
        self.init_timestamp = -1.0
        self.init_timestamp_flag = False
        self.tasks_info = TasksRequest()
        self.attack_info_string = AttackRequest()
        # list of dict(stages). dict: key enemy, value: ally.
        self.attack_stages_info = []
        self.pub_tasks_ei = rospy.Publisher('ExecInfoRequest', ExecInfoRequest, queue_size=1)
        self.attack_direction = None
        self.rate = rospy.Rate(1)

        self.time_array = []
        self.n_tasks = 2


    def initial(self):
        
        # 等待甘特图输入
        while not rospy.is_shutdown():
            if not self.init_timestamp_flag:
                rospy.logwarn('Task Request not received!')
                self.rate.sleep()
            else:
                # 解析车辆开始时间
                print('Received task request')
                self.parse_task_info()
                break

        return 0

    def task_request_callback(self, msg: TasksRequest):
        if not self.init_timestamp_flag:
            self.init_timestamp_flag = True
            self.init_timestamp = msg.init_timestamp

        self.tasks_info = msg

    def parse_task_info(self, msg: TasksRequest):
        ''' 仅仅解析各个任务的开始时间。汽研所场景简化
        '''
        if len(self.time_array) !=0:
            print('parse already')
            return
        for task in self.tasks_info.tasks:
            self.time_array.append(task.start_time)
        ta = np.array(self.time_array)
        self.time_array = np.sort(ta)
   
    def attack_request_callback(self, msg):
        self.attack_info_string = msg
        self.parse_attack_info()

    def parse_attack_info(self):
        # translate the attack string into list of dict. (one ally can attack one ally only)
        self.attack_stages_info = []
        for string in self.attack_info_string:
            attack_pairs = string.split('*')
            stage = {}
            for attack_pair in attack_pairs:
                enemy_id = int(attack_pair[2:])
                ally_id = int(attack_pair[:2])
                if enemy_id in stage.keys():
                    stage[enemy_id].append(ally_id)
                else:
                    stage[enemy_id] = [ally_id]
            self.attack_stages_info.append(stage)
        
        # 根据数组初步判断。看第一个阶段打哪辆车.  TODO 确认敌方第一辆车是4号车。
        
        if 4 in self.attack_stages_info[0].keys():
            self.attack_direction = -1          # 4打左边的车
        elif 5 in self.attack_stages_info[0].keys():
            self.attack_direction = 1           # 5打右边的车
        else:
            rospy.logerr('cannot define attack direction')

    def ei_feedback(self, task_now, t_remain):
        ''' 发布剩余时间

        '''
        # 假设order按照顺序发送。忽略排序的过程
        exec_info = ExecInfoRequest()

        # time_now = time.time() - self.init_timestamp
        for task_info in self.tasks_info.tasks:
            task_ei = TaskEI()
            
            task_ei.order = task_info.order
            tid = task_info.order
            if tid >task_now:
                task_ei.status = -1
            elif tid == task_now:
                task_ei.status = 1
                task_ei.duration_upd = t_remain         # 从当前时刻开始的剩余时间
            elif tid < task_now:
                task_ei.status = 0
            
            # task_ei.duration_upd = task_time[task_now] - time_now 
            exec_info.tasks_ei.append(task_ei)

        self.pub_tasks_ei(exec_info)
        

    def judge_next_task(self, pretask):
        ''' 根据TasksRequest开始信息进行。
        '''
        assert self.init_timestamp_flag, 'task not init. donot have init_timestamp'
        time_now = time.time() - self.init_timestamp
        res = True
        if time_now < self.tasks_info.tasks[pretask+1].start_time:
            res=False
        return res
    


# if __name__ == '__main__':

