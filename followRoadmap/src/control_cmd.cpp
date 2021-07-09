#include "FollowRoadMap.h"
#include <fstream>
#include <std_msgs/Int16MultiArray.h>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <stdlib.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_cmd");

    string filePath("/home/tiecun/catkin_ws/src/trajectory_tracking/roadmap/roadMap04012125.txt");
    
    FollowRoadMap followRM(filePath, true);

    if (!followRM.loadRoadPath())
    {
        return -1;
    }

    // followRM.waitForInit();

    followRM.run();

    return 0;
}