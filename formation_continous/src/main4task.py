#!/usr/bin/env python3

"""
v 1.0
main程序。完成一系列 规划任务。
"""

from typing import List, Tuple

import rospy
import time

from formation_common.config_formation_continous import *

from formation_common.formation_zoo import *
from ros_interface import  ROSInterface,  TaskManager

from formation_trajectory_planning import  MAFormationTrajPlanning

from load_task_points import load_task_configs

class TaskExcute(ROSInterface):
    def __init__(self, n_car, car_colors, fs_lists):
        super().__init__(n_car, car_ids)
        # 加载任务
        self.n_task = len(car_colors)
        n_task = self.n_task
        self.ftps= []
        for i in range(n_task):
            ftp = MAFormationTrajPlanning(car_colors[i], fs_lists[i])
            self.ftps.append(ftp)

        self.prate = rospy.Rate(2)     # planing rate

    def initial(self, task_manager:TaskManager=None):
        if task_manager is not None:
            task_manager.initial()
        super().initial()

    def running(self, tm:TaskManager=None):
        i_task = 0
        if tm is None:
            t_array = np.ones([self.n_task])*-1
        else:
            t_array = tm.time_array

        isFeedBack = True           #
        while not rospy.is_shutdown():
            if i_task == self.n_task:
                print('all task finished!')
                break
            ftp = self.ftps[i_task]
            t_now = time.time()
            if t_now > t_array[i_task]:
                if i_task == self.n_task - 1:       # 最后一个任务用顺序通行的方法
                    obs = [1]
                else:
                    obs=[]
                # obs = []
                trajs, t_remain = ftp.traj_planning(self.pose_states, obs)

                print('remaining time:' , t_remain)
                if t_remain<0:
                    i_task +=1
                    continue

                self.publish_trajs(trajs)
                # 辅助性画图工具
                self.publish_csps(ftp.individual_csps)
                if isFeedBack and tm is not None:
                    tm.ei_feedback(i_task, t_remain)
                isFeedBack = not isFeedBack
                self.prate.sleep()

if __name__=='__main__':

    car_color_list, fs_list = load_task_configs()
    
    task_main = TaskExcute(n_car, car_color_list, fs_list)
    tm = TaskManager()
    # with task feedback
    # task_main.initial(tm)
    # task_main.running(tm)
    task_main.initial()
    task_main.running()