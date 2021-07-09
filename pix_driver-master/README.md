# __pixloop drive_by_wire chassis ROS driver__

contact wtih <mark@pixmoving.com>

## software and hardware requirement
* 1. ubuntu 18.04, ROS melodic
* 2. the driver support peakcan and socketcan, so can card drivers are required
* 3. python-can third-party library is required (install python-can by type in this commannd `pip install python-can`)

## __instruction__
* 1. launch pix_driver_write.launch to conntrol pixloop chassis. 
* this driver subscribe topic /control_cmd, convert control_cmd into Int16MultiArray message with frame id and can message, then publish /canbus_message as type Int16MultiArray

| launch param| default  | detail |
| --------   | -----:  | ----- |
| can_type | socketcan | can type，pcan or socketcan |
| can_channel | can0 | can channnel |
have a look at <https://python-can.readthedocs.io/en/master/>

* 2. turn on the feedback function by launching pix_driving_read.launch.
* this node decodes canbus message with frame id 0x193 from chassis, then publish feedback message with topic /pix_chassis_feedback

| launch param| default  | detail |
| --------   | -----:  | ----- |
| can_type | socketcan | can type，pcan orsocketcan |
| can_channel | can0 | can channnel |
have a look at <https://python-can.readthedocs.io/en/master/>

* 3. see detailed structure of /control_cmd in control_cmd_example.txt

## __driver installation__


1. copy package into your ROS workspack：`cp -r pix_driver [your_ws]/src`

2. compile the workspace：`cd your_ws && catkin_make`

3. add environment variable: ` echo "source [your_ws]/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc `


## __driver usage__

1. launch chassis control function：`roslaunch pix_driver pix_driver_write.launch`

2. launch chassis feedback function：`roslarunch pix_driver pix_driver_read.launch`

------
## __topic structure__
## /control_cmd message structure
> /control_cmd is a topic which conntrol node subscribe，it publish messages as the type of Int16Multiarray

#### 1. the bound of physical value of speed is 0-60km/h，the bound of signal value is 0-600
#### 2. the bound of physical value of steering is -30 degree to 30 degree，the bound of signal value is -1024 - +1024
#### 3. the data in index 8 is self driving enable switch
#### 4. the data inn index 11 is steering mode，1: unsynch、2: conventional、3: synch


| index        | conttol signal  | detail |
| --------   | -----:  | ----- |
| 0 |  speed | speed value，0 to 600(0-60KM/H)|
| 1 |  steering| steering value,-1024 to 1024（plus means right，minus means left,steering boundary is -30 degree to +30 degree|
| 2 | braking| braking velue0 to 1024|
| 3 | gear | Gear shift 1:D 2:N 3:R |
| 4 | EPB | 0:EPB disable 1:EPB enable |
| 5|  right light| 0: right turning sight off 1: right turning sight on |
| 6| left_light | 0: left turning sight off 1: left turning sight on|
| 7|  front_light| 0:front light on 1:front light off |
| 8| self driving enable | 0:self driving disable   1:self driving enable |
| 9| speed mode | 0:normal speed（max speed: 60km/h）  1:low speed（max speed: 5km/h）|
| 10| advanced mode | 0:basic mode 1:advanced mode |
| 11| mode selection | 1:unsynch 2:conventional 3:synch|
| 12| state control | 0;normal 1:emergency |


#### /canbus_message message structure
| index        | signal |
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

### /pix_chassis_feedback message structure
| type | name | detail |
| --------   | -----  | ----|
| Header | header | sequence、time stamp
| float64 | Speed_feedback | speed feedback  -600 to 600    Unit：0.1KM/h|
| float64 | F_steer_feedback | unit：degree（steering wheel angle）|
| float64 | Braking_feedback | braking feedback  0-1000  Unit：0.01Mp|
| int8 | Gear_feedback | gear feedback 0x1:D 0x2：N 0x3：R|
| int8 | mode_feedback | steering mode 1-unsynch 2-conventional 3-synch|
| bool | L_steer_light_feedback | 0-left turning sight off 1-left turning sight on|
| bool | R_steer_light_feedback | 0-right turning sight off 1-right turning sight on|
| bool | Tail_light_feedback | tail light feedback 0：of 1：on|
| bool | Braking_light_feedback| braking light feedback 0：off 1：on|
| bool | Vehicle_status_feedback| vehicle status 0：normal 1：abnormal|
| bool | Vehicle_mode_feedback| vehicle mode feedback 0：remote mode 1：self driving mode|
| bool | Emergency_stop_feedback| emergency stop feedback 0：normal 1：emergency|
