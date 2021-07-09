// author: Yinghan
/*
# Naming Rule
file: lower case with "_", eg. file_file.txt
variable: lower case with "_", eg. int var_a, if global, with "g_", eg. float g_index
class: upper case on first letter of each word, without "_", eg. class MyClass
Function: upper case on first letter of each word, without "_". prefer verb.. eg. OpenFile()
ENUM: all upper case, eg. PI
struct: similar to ENUM or class
*/

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
//#include <sstream>
//#include <string>
//#include <vector>

using namespace std;

//*********** variable and struct ************
// 平面坐标
struct XYZCOORD
{
    double x;
    double y;
    double z;
};

//大地坐标系（可以是 北京54坐标系，西安80坐标系，WGS84坐标系（GPS 坐标））
struct GPSCOORD
{
    double longitude;  //经度
    double latitude;   //纬度
    double height;     //大地高,可设为0
};

extern GPSCOORD gps_coord;
extern XYZCOORD xyz_coord;

// dataStruct of RoadPoint
struct RoadPoint
{
    uint32_t roadId = 0;
    uint32_t index = 0;
    uint32_t totalIndex = 0;
    double x;
    double y;
    double courseAngle = 0;
    int16_t roadType = 0;
    double curvature = 0;
    double suggest_vel = 0;
    bool b_onRoadPoint = false;
    bool b_findPreviewPoint = false;
};

struct DecisionData
{
    // RoadPoint currentPosture;  // real point
    RoadPoint currentPoint;  // point in roadmap
    RoadPoint previewPoint;
    double targetVelocity = 0;
    double targetAngleVel = 0;
    double previewDistance = 0;
    uint8_t currentState = 0;  //代表是否找到预瞄点，0未找到，1找到
    int32_t nextId = 0;
    int32_t nextNextId = 0;
};

#define MIN_DISTANCE 1
#define DISTANCE_RANGE 0.02
#define MAX_PREVIEW_DISTANCE_DELTA 0.1

/*
#define WGS84 84     //WGS84坐标系（GPS 坐标）
#define BJ54 54      //北京54坐标系
#define XIAN80 80    //西安80坐标系
#define ZONEWIDE3 3  //投影带宽度 3
#define ZONEWIDE6 6  //投影带宽度 6
*/

// convert GPS coordinate to XYZ coordinate
// Datum must be 84/54/80, zonewide must be 3/6
//void ConvertGPS2XYZ(GPSCOORD &pcg, XYZCOORD &pcc, int Datum, int zonewide);

//void ProcessGPSMsg(const sensor_msgs::NavSatFix msg);

/*
// split string by space, return a vector
vector<string> SplitStrBySpace(string str)
{
    vector<string> res;

    string result;
    stringstream input(str);

    while (input >> result)
        res.push_back(result);

    return res;
}

// convert string to number
template <class Type> Type ConvertStr2Num(const string &str)
{
    istringstream iss(str);
    Type num;
    iss >> num;

    return num;
}

// compute distance between (x1,y1) and (x2,y2)
template <class Type> double ComputeDistance(Type x1, Type y1, Type x2, Type y2)
{
    Type distance = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

    return distance;
}
*/