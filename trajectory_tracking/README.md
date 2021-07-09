# 轨迹跟踪模块说明
1. 接收话题
   - /local_trajectory（格式/trajectory_tracking/Trajectory）
   - /gps（格式 /nav_msgs/Odometry）
2. 发布话题/control_cmd（格式Int16MultiArray）
3. 使用方法 
    - **实车**
        ```bash
        roslaunch trajectory_tracking pure_pursuit_python.launch 
        ```
    - **仿真**
        ```bash
        roslaunch trajectory_tracking pure_pursuit_simulation.launch
        roslaunch simulation simulation.launch
        ```
4. 原理：目前使用纯追踪算法控制转角，速度按照局部轨迹中最近的点的速度发布

