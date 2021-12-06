# ……
1. 各程序包说明
- followRoadmap 最原始的循迹代码。读取采集的txt(gnss_gpchc_driver/map)作为输入，接收GPS信号，发送底盘控制信号
- gnss_gpchc_driver GNSS驱动包。使用节点 gnssrtk_node。读取串口数据，输出nav_msgs/Odometry数据。/map文件夹保存采集的路点文件
- pix_driver-master PIX提供底盘驱动程序。使用Int16Multiarray结构的/control_cmd话题进行控制底盘。详细协议看pix_driver-master下的README-zh.md文件
- simulation 仿真包。接收/control_cmd控制虚拟车辆，默认开始在道路开始点，发布 nav_msgs/Odometry位置信息。并将相应的车辆画出来。
- trajectory_tracking 轨迹跟踪模块。输入局部轨迹(路径+速度)，输出控制命令/control_cmd。使用pure_pursuit.py


2. 仿真demo
- demo 
 1. 单车轨迹跟踪仿真
```bash
trajectory_tracking pure_pursuit_simulation.launch
simulation simulation_pp.launch
```
2. 多车协同仿真
```bash
rosrun formation_dec forRealCar.py
roslaunch simulation multi_car_simulation.launch
```

3. IP列表
   1. 小车4： 192.168.1.88