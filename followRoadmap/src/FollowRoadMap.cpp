#include "FollowRoadMap.h"
#include <fstream>

FollowRoadMap::FollowRoadMap(string &roadMapFilePath, bool isUsedROSParam) : roadMapFilePath(roadMapFilePath), rate(60)
{
    if(isUsedROSParam)
        nh.getParam("/roadmap_path", this->roadMapFilePath) ;

    pub_vel = nh.advertise<std_msgs::Int16MultiArray>("control_cmd", 1);
    // pub_currentPose = nh.advertise<geometry_msgs::Pose>("currentPose", 1000);
    sub_pose = nh.subscribe<nav_msgs::Odometry>("gps", 10, boost::bind(&FollowRoadMap::computeVelocity, this, _1));
    pub_preview_point = nh.advertise <geometry_msgs::Point>("purepusuit/preview_point", 1);

}

FollowRoadMap::~FollowRoadMap() {}

bool FollowRoadMap::loadRoadPath()
{
    ifstream infile(roadMapFilePath, ios::in);

    if (infile.is_open())
    {
        RoadPoint roadPoint;
        vector<RoadPoint> temp;
        uint32_t lastId;
        uint32_t index = 0;
        uint32_t totalIndex = 0;
        string s;
        double x;
        double y;

        // vector<double> x_coordinate;
        // vector<double> y_coordinate;

        if (getline(infile, s))
        {
            istringstream is(s);

            is >> roadPoint.roadId;
            lastId = roadPoint.roadId;
            is >> x;
            is >> y;

            roadPoint.x = x;
            roadPoint.y = y;

            roadPoint.index = 0;
            roadPoint.totalIndex = 0;

            // x_coordinate.push_back(x);
            // y_coordinate.push_back(y);

            is >> roadPoint.courseAngle >> roadPoint.roadType >> roadPoint.curvature >> roadPoint.suggest_vel;

            temp.push_back(roadPoint);

            s.clear();
        }
        else
        {
            ROS_ERROR_STREAM("roadMap roadpoints error! Please check the txt file!");
            return false;
        }

        while (getline(infile, s))
        {
            istringstream is(s);
            is >> roadPoint.roadId;
            is >> x;
            is >> y;

            roadPoint.x = x;
            roadPoint.y = y;
            totalIndex++;
            roadPoint.totalIndex = totalIndex;

            // x_coordinate.push_back(x);
            // y_coordinate.push_back(y);

            is >> roadPoint.courseAngle >> roadPoint.roadType >> roadPoint.curvature >> roadPoint.suggest_vel;

            if (roadPoint.roadId != lastId)
            {
                index = 0;
                roadPoint.index = 0;
                roadPoints.push_back(temp);
                temp.clear();
            }
            else
            {
                index++;
                roadPoint.index = index;
            }

            temp.push_back(roadPoint);

            lastId = roadPoint.roadId;

            s.clear();
        }

        roadPoints.push_back(temp);

        infile.close();

        // nh.setParam("roadPoints_x", x_coordinate);
        // nh.setParam("roadPoints_y", y_coordinate);
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("roadMap file opening failed! Please check the path of the file!");
        return false;
    }
}

double FollowRoadMap::pointDistance(double x0, double y0, double x1, double y1)
{
    double delta_x = x1 - x0;
    double delta_y = y1 - y0;
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

void FollowRoadMap::computeVelocity(const nav_msgs::Odometry::ConstPtr &msg)
{
    currentPose.position = msg->pose.pose.position;

    double vel_x = msg->twist.twist.linear.x;
    double vel_y = msg->twist.twist.linear.y;
    double currentVelocity = sqrt(vel_x * vel_x + vel_y * vel_y);

    double yaw = msg->twist.twist.angular.z;

    ROS_INFO_STREAM("currentVel is " << currentVelocity << ", yaw is " << yaw);

    /**************************Find the currentPoint****************************/
    uint32_t roadPointssSize = static_cast<uint32_t>(roadPoints.size());
    uint32_t lastId = decisionData.currentPoint.roadId;
    uint32_t nextIdFromLast = getNextRoadId(lastId, roadPointssSize - 1);
    RoadPoint currentPoint =
        getCurrentPoint(currentPose.position.x, currentPose.position.y, roadPoints, lastId, nextIdFromLast);

    ROS_INFO_STREAM("currentPoint ID is " << currentPoint.roadId << ", index is " << currentPoint.index);
    ROS_INFO_STREAM("currentPoint X is " << currentPoint.x << ", Y is " << currentPoint.y);
    double currentDistance = pointDistance(currentPose.position.x, currentPose.position.y, currentPoint.x, currentPoint.y);
    ROS_INFO_STREAM("currentDistance is " << currentDistance);
    /*********************End of Finding the currentPoint***********************/

    if (currentPoint.b_onRoadPoint)
    {
        decisionData.currentPoint = currentPoint;

        decisionData.nextId = getNextRoadId(currentPoint.roadId, roadPointssSize - 1);
        decisionData.nextNextId = getNextRoadId(decisionData.nextId, roadPointssSize - 1);

        static double lastPreviewCurvature = 0;
        // double maxCurvature = fmax(fabs(lastPreviewCurvature), fabs(currentPoint.curvature));
        // getPreviewDistance(currentVelocity, fabs(currentPoint.curvature), decisionData);
        decisionData.previewDistance = 1;
        RoadPoint previewPoint = getPreviewPoint(decisionData, roadPoints);
        previewPointPub.x = previewPoint.x;
        previewPointPub.y = previewPoint.y;

        double previewDistance_real =
            pointDistance(currentPose.position.x, currentPose.position.y, previewPoint.x, previewPoint.y);

        ROS_INFO_STREAM("previewDistance_real is " << previewDistance_real);
        ROS_INFO_STREAM("PreviewDistance is " << decisionData.previewDistance);
        ROS_INFO_STREAM("previewPoint ID is " << previewPoint.roadId << ", index is " << previewPoint.index);
        ROS_INFO_STREAM("previewPoint X is " << previewPoint.x << ", Y is " << previewPoint.y);

        double delta_x = -(previewPoint.x - currentPose.position.x) * sin(yaw * M_PI / 180) +
                         (previewPoint.y - currentPose.position.y) * cos(yaw * M_PI / 180);

        ROS_INFO_STREAM("delta_x is " << delta_x);

        double previewCurvature = 2 * delta_x / pow(previewDistance_real, 2);
        lastPreviewCurvature = previewCurvature;
        ROS_INFO_STREAM("previewCurvature is " << previewCurvature);

        double commandVel = 0;

        if (fabs(previewCurvature) <= 0.1)
        {
            commandVel = -velK1 * fabs(previewCurvature) + velcrv0;
        }
        else
        {
            commandVel = -velK1 * fabs(previewCurvature) + velcrv1 + velK1 * 0.1;
        }

        if (commandVel > currentPoint.suggest_vel)
        {
            commandVel = currentPoint.suggest_vel;
        }

        if (commandVel - currentVelocity > 0.2)
        {
            ROS_INFO_STREAM("overspeed currentVelocity = " << currentVelocity);
            
            commandVel = currentVelocity + 0.2;
        }
        else if (commandVel - currentVelocity < -0.2)
        {
            ROS_WARN("slowspeed");
            commandVel = currentVelocity - 0.2;
        }

        if (commandVel < 0.3)
        {
            commandVel = 0.3;
        }
        ROS_INFO_STREAM("test here");

        control_cmd.data[0]=(int(commandVel * 36));

        control_cmd.data[3] = 1;      // 3 gear : D
        
        control_cmd.data[8] = 1;        //8
        control_cmd.data[9]=1;          // 9 low speed mode


        ROS_INFO_STREAM("commandvel is " << commandVel);


        double angle =   0.8 *  previewCurvature * 180/ M_PI;
        if (fabs(angle) > 30)
        {
            angle = copysign( 30, angle);
        }

        control_cmd.data[1] = (int)(angle * 1024 / 30);
                ROS_INFO_STREAM("control_cmd.data "<<control_cmd.data[0]<<" "<<control_cmd.data[1]<<" "<<control_cmd.data[3]<<" "<<control_cmd.data[8]<<" "<<control_cmd.data[9]<< endl);
        ROS_INFO_STREAM("steeringAngle is " << angle  << endl << endl);
    }
    else
    {
        ROS_ERROR_STREAM("can't find the currentPoint on the Road, scout has to stop!");
        control_cmd.data[0]=0;
        control_cmd.data[1]=0;

    }
}

uint32_t FollowRoadMap::getNextRoadId(uint32_t currentId, uint32_t maxId)
{
    if (currentId == maxId)
    {
        currentId = maxId;
    }
    else
    {
        currentId++;
    }

    return currentId;
}

RoadPoint FollowRoadMap::getCurrentPoint(double x, double y, const vector<vector<RoadPoint>> &roadPoints_, uint32_t lastId,
                                         uint32_t nextId)
{
    RoadPoint ret;
    uint32_t minPointId_L = 0;
    uint32_t minPointId_N = 0;
    double minDistance_L = MIN_DISTANCE;
    double minDistance_N = MIN_DISTANCE;
    double distTemp_L = 0;
    double distTemp_N = 0;

    for (uint32_t i = 0; i < roadPoints_[lastId].size(); i++)
    {
        distTemp_L = pointDistance(x, y, roadPoints_[lastId].at(i).x, roadPoints_[lastId].at(i).y);
        if (distTemp_L < minDistance_L)
        {
            minDistance_L = distTemp_L;
            minPointId_L = i;
        }
    }

    if (lastId != nextId)
    {
        for (uint32_t i = 0; i < roadPoints_[nextId].size(); i++)
        {
            distTemp_N = pointDistance(x, y, roadPoints_[nextId].at(i).x, roadPoints_[nextId].at(i).y);
            if (distTemp_N < minDistance_N)
            {
                minDistance_N = distTemp_N;
                minPointId_N = i;
            }
        }
    }
    ROS_INFO_STREAM("minDistance_L="<<minDistance_L<<", "<<"minDistance_N="<<minDistance_N);
    if ((minDistance_L < minDistance_N) && (minDistance_N <= MIN_DISTANCE))
    {
        ROS_INFO_STREAM("x is " << x << ", y is " << y << ", minDistance_L is " << minDistance_L);
        ret = roadPoints_[lastId].at(minPointId_L);
        ret.b_onRoadPoint = true;
    }
    else if ((minDistance_N < minDistance_L) && (minDistance_L <= MIN_DISTANCE))
    {
        ROS_INFO_STREAM("x is " << x << ", y is " << y << ", minDistance_N is " << minDistance_N);
        ret = roadPoints_[nextId].at(minPointId_N);
        ret.b_onRoadPoint = true;
    }
    else
    {
        ret.b_onRoadPoint = false;
    }

    return ret;
}

void FollowRoadMap::getPreviewDistance(double velocity, double curvature, DecisionData &decisionData)
{
    double previewDistance = 0;
    static double lastPreviewDistance;
    double k = 5;
    double a = 0.1;

    previewDistance = k * velocity * exp(-a * curvature);

    if (previewDistance - lastPreviewDistance > MAX_PREVIEW_DISTANCE_DELTA)
    {
        previewDistance = lastPreviewDistance + MAX_PREVIEW_DISTANCE_DELTA;
    }
    else if (lastPreviewDistance - previewDistance > MAX_PREVIEW_DISTANCE_DELTA)
    {
        previewDistance = lastPreviewDistance - MAX_PREVIEW_DISTANCE_DELTA;
    }

    if (previewDistance < 0.1)
    {
        previewDistance = 0.1;
    }

    lastPreviewDistance = previewDistance;
    decisionData.previewDistance = previewDistance;
}

RoadPoint FollowRoadMap::getPreviewPoint(const DecisionData &decisionData, const vector<vector<RoadPoint>> &roadPoints_)
{
    RoadPoint previewPointCandidate;
    RoadPoint currentPoint_ = decisionData.currentPoint;
    uint32_t currentId = decisionData.currentPoint.roadId;
    uint32_t nextId = decisionData.nextId;
    uint32_t nextNextId = decisionData.nextNextId;

    double distance = decisionData.previewDistance;
    double distanceFound;

    for (uint32_t index = currentPoint_.index; index < roadPoints_[currentId].size(); index++)
    {
        distanceFound =
            pointDistance(currentPoint_.x, currentPoint_.y, roadPoints_[currentId][index].x, roadPoints_[currentId][index].y);

        if (distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE)
        {
            previewPointCandidate = roadPoints_[currentId][index];
            previewPointCandidate.b_findPreviewPoint = true;
        }
    }

    if ((!previewPointCandidate.b_findPreviewPoint) && (currentPoint_.roadId != nextId))
    {
        for (uint32_t index = 0; index < roadPoints_[nextId].size(); index++)
        {
            distanceFound =
                pointDistance(currentPoint_.x, currentPoint_.y, roadPoints_[nextId][index].x, roadPoints_[nextId][index].y);

            if (distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE)
            {
                previewPointCandidate = roadPoints_[nextId][index];
                previewPointCandidate.b_findPreviewPoint = true;
            }
        }
    }

    if ((!previewPointCandidate.b_findPreviewPoint) && (nextId != nextNextId))
    {
        for (uint32_t index = 0; index < roadPoints_[nextNextId].size(); index++)
        {
            distanceFound = pointDistance(currentPoint_.x, currentPoint_.y, roadPoints_[nextNextId][index].x,
                                          roadPoints_[nextNextId][index].y);

            if (distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE)
            {
                previewPointCandidate = roadPoints_[nextNextId][index];
                previewPointCandidate.b_findPreviewPoint = true;
            }
        }
    }

    if (!previewPointCandidate.b_findPreviewPoint)
    {
        ROS_ERROR_STREAM("can't find previewPoint, choose the end of the path as it");
        previewPointCandidate = roadPoints_[nextNextId].back();
    }

    return previewPointCandidate;
}

void FollowRoadMap::waitForInit()
{
    while (!nh.hasParam("scoutState"))
    {
        usleep(100000);
    }

    return;
}

void FollowRoadMap::run()
{
    for(int i =0;i<13;i++){
        control_cmd.data.push_back(0);
    }
    while (ros::ok())
    {
        ros::spinOnce();

        /*nh.getParam("scoutState", state);
        if (!state)
        {
            ros::shutdown();
        }*/

        pub_vel.publish(control_cmd);
        pub_preview_point.publish(previewPointPub);


        // pub_currentPose.publish(currentPose);

        rate.sleep();
    }
}
