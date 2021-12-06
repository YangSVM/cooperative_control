# __pixloop线控底盘自动驾驶ROS驱动__

详情咨询<mark@pixmoving.com>

## 使用前满足条件
* 1. 支持peakcan与socketcan，peakcan驱动请从peak官网中下载，并按照官方使用说明安装
* 2. 确保python环境中有python-can第三方库(安装方式 `pip install python-can`)

## __简介__
* 1. 通过启动pix_driver_write.launch文件开启底盘控制功能
该驱动节点订阅/control_cmd话题，将控制量转化成为带有帧id、can消息的Int16MultiArray类型的消息，以/canbus_message话题发布Int16MultiArray类型的消
息，同时通过can卡向底盘发送can消息控制底盘

| launch 参数| 默认值  | detail |
| --------   | -----:  | ----- |
| can_type | socketcan | can类型，现支持pcan与socketcan |
| can_channel | can0 | can通道 |
详情可以参考<https://python-can.readthedocs.io/en/master/>

* 2. 通过启动pix_driver_read.launch文件开球底盘反馈接收功能，该节点通过解析底盘帧ID为0x193的can消息，以/pix_chassis_feedback话题发布底盘反馈

| launch 参数| 默认值  | detail |
| --------   | -----:  | ----- |
| can_type | socketcan | can类型，现支持pcan与socketcan |
| can_channel | can0 | can通道 |
详情可以参考<https://python-can.readthedocs.io/en/master/>

* 3. 具体字段定义请参考/control_cmd 结构与 control_cmd_example.txt

## __driver安装__


1. 复制package进入你的ROS工作空间：`cp -r pix_driver [your_ws]/src`

2. 编译工作空间：`cd your_ws && catkin_make`

3. 加入系统全局变量: ` echo "source [your_ws]/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc `


## __driver使用__

1. 启动底盘控制节点：`roslaunch pix_driver pix_driver_write.launch`

2. 启动底盘状态反馈节点：`roslarunch pix_driver pix_driver_read.launch`

------
## __话题结构__
## /control_cmd 结构
> /control_cmd 是底盘控制节点所订阅的topic，该节点的message是一个Int16Multiarray，各个索引所对应的控制信息如下

#### 1. 速度speed物理值区间为0-60km/h，信号区值区间为0-600
#### 2. 转向steering物理值区间为左30度至右30度，信号值区间为-1024-+1024
#### 3. 8号字段为自动驾驶使能失能信号
#### 4. 11号字段为转向模式选择，1为前后轮异向、2为传统模式、3为前后轮同向 


| index        | 控制信号  | detail |
| --------   | -----:  | ----- |
| 0 |  speed | speed value，0 to 600(0-60KM/H)|
| 1 |  steering| steering value,-1024 to 1024（右为正，左为负，转向区间是-30度-30度|
| 2 | braking| 刹车值0 to 1024|
| 3 | gear | Gear shift 0x1-D 0x2-N 0x3-R |
| 4 | EPB | 0-电子驻车关 1-电子驻车开 |
| 5|  right light| 0-右转向灯关 1-右转向灯开 |
| 6| left_light | 0-左转向灯关 1-左转向灯开|
| 7|  front_light| 0-大灯关 1-大灯开 |
| 8| self driving enable | 0-自动驾驶失能   1-自动驾驶使能 |
| 9| speed mode | 0-常速模式（限速60km/h）  1- 低速模式（限速5km/h）|
| 10| advanced mode | 0-基础模式 1-高级模式 |
| 11| mode selection | 0x1-异向 0x2-传统模式 0x03-同向|
| 12| state control | 0-正常 1-急停 |


#### /canbus_message 结构
| index        | 信号  |
| --------   | -----:  | 
| 0 |  frame ID| 
| 1 | byte 0|
| 2 | byte 1|
| 3 | byte 2|
| 4 | byte 3|
| 5 | byte 4|
| 6 | byte 5|
| 7 | byte 6|
| 8 | byte 7|

### /pix_chassis_feedback 结构
| type | name | detail |
| --------   | -----  | ----|
| Header | header | 编号、时间戳
| float64 | Speed_feedback | 车速反馈  -600 to 600    单位：0.1KM/h|
| float64 | F_steer_feedback | 单位：度（方向盘转角）|
| float64 | Braking_feedback | 制动值反馈  0-1000  单位：0.01Mp|
| int8 | Gear_feedback | 当前挡位 0x1:D档 0x2：N档 0x3：R档|
| int8 | mode_feedback | 当前转向模式 0x1:前后异向模式 0x2:常规模式 0x3：前后同向模式|
| bool | L_steer_light_feedback | 左转向灯状态 0：关 1：开|
| bool | R_steer_light_feedback | 右转向灯状态 0：关 1：开|
| bool | Tail_light_feedback | 尾灯灯状态 0：关 1：开|
| bool | Braking_light_feedback| 制动灯状态 0：关 1：开|
| bool | Vehicle_status_feedback| 车辆状态 0：正常 1：异常|
| bool | Vehicle_mode_feedback| 车辆模式 0：遥控模式 1：自动驾驶模式|
| bool | Emergency_stop_feedback| 急停状态 0：正常 1：急停|
