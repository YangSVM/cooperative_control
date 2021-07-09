#ifndef FOLLOWROADMAP_H
#define FOLLOWROADMAP_H

#include "tools.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <string>

using namespace std;

class FollowRoadMap
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub_pose;
    ros::Publisher pub_vel, pub_currentPose, pub_preview_point;
    ros::Rate rate;

    bool state;

    vector<vector<RoadPoint>> roadPoints;

    const double v_max = 1.5;     // m/s
    const double w_max = 0.7853;  // rad/s

    const double velcrv0 = 1.2;  // speed at curvature 0
    const double velcrv1 = 0.7;  // speed at curvature 0.1
    const double velcrv2 = 0.4;  // speed at curvature 0.3
    const double velK1 = (velcrv0 - velcrv1) / (0.1 - 0);
    const double velK2 = (velcrv1 - velcrv2) / (0.3 - 0.1);

    DecisionData decisionData;

    geometry_msgs::Point previewPoint;
    geometry_msgs::Point currentPoint;
    geometry_msgs::Pose currentPose;
    std_msgs::Int16MultiArray control_cmd; 

  public:
    FollowRoadMap(string &roadMapFilePath, bool isUsedROSParam=true);
    ~FollowRoadMap();

    string roadMapFilePath;
    geometry_msgs::Point previewPointPub;

    double pointDistance(double x0, double y0, double x1, double y1);

    // load the path to follow
    bool loadRoadPath();

    uint32_t getNextRoadId(uint32_t currentId, uint32_t maxId);

    RoadPoint getCurrentPoint(double x, double y, const vector<vector<RoadPoint>> &roadPoints_, uint32_t lastId,
                              uint32_t nextId);

    void getPreviewDistance(double velocity, double curvature, DecisionData &decisionData);

    RoadPoint getPreviewPoint(const DecisionData &decisionData, const vector<vector<RoadPoint>> &roadPoints_);

    // Compute velocity commands each time new pose data is received.
    void computeVelocity(const nav_msgs::Odometry::ConstPtr &msg);

    // Run the controller.
    void run();

    // Wait for the initialization of scout
    void waitForInit();
};
#endif
