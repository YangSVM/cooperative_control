# 多车调试方法
1. ROS多机主从模式： https://www.cnblogs.com/liu-fa/p/5773822.html
2. remap： 
每辆车都需要启动GPS,启动底盘，启动轨迹跟踪模块。添加:
  <group ns="car1">
	<node ...> </node>
  </group>
会将所有的node、话题添加car1的前缀避免混淆

3. 启动 trajectory_tracking.launch:启动所有车辆的 轨迹跟踪模块，gnss模块，底盘控制模块。
