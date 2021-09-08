cmake_minimum_required(VERSION 3.0.2)
project(trajectory_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	)


add_message_files(
  FILES
  RoadPoint.msg
  Trajectory.msg

)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pix_driver
 CATKIN_DEPENDS message_runtime 
#  DEPENDS system_lib
)

 catkin_install_python(PROGRAMS
   src/record.py
   src/pure_pursuit.py
   src/pub_txt_road_points.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

