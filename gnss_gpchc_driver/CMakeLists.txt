cmake_minimum_required(VERSION 2.8.3)
project(gnss_gpchc_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  nmea_msgs
  nav_msgs
  sensor_msgs 
)
catkin_package(
#  INCLUDE_DIRS include
   CATKIN_DEPENDS  
#    message_runtime 
    nmea_msgs
    nav_msgs
    sensor_msgs
    serial
#  DEPENDS system_lib
)

include_directories(
  .
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


# add_executable( gnssrtk_node_ori 
#   src/GPS_Subscriber_GPYBM.cpp
# ) 

add_executable( gnssrtk_node
  src/main.cpp
) 

# add_executable( gnssrtk_node_right
#   src/GPS_Subscriber_Right.cpp
# ) 

# add_executable( gnss_transfer
#   src/gnss_transfer.cpp
# ) 

#add_dependencies(gnssrtk_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
# target_link_libraries( gnssrtk_node_ori 
#   ${catkin_LIBRARIES}
#   udev
# )

target_link_libraries( gnssrtk_node 
  ${catkin_LIBRARIES}
  udev
)

# target_link_libraries( gnssrtk_node_right 
#   ${catkin_LIBRARIES}
#   udev
# )

# target_link_libraries( gnss_transfer 
#   ${catkin_LIBRARIES}
#  udev 
# )