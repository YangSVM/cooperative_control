#include <fstream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <math.h>
#include<cmath>


#define SAVE_ROAD_FLAG true
#define PI 3.141592653589793
#define E 0.006693422
#define A 6378245

using namespace std;
/*
输出同一使用右手系：
输出话题名称，格式，变量名
/gps    :nav_msgs::Odometry     gps_msg  
                gps_msg.pose.pose.position.x                    正东正方向
                gps_msg.pose.pose.position.y                    正北正方向
                gps_msg.twist.twist.angular.z                      正东为0，逆时针为正。角度制。
                gps_msg.twist.twist.linear.x                          东向速度
                gps_msg.twist.twist.linear.y                          北向速度

保存文件。角度制。坐标系相同。文件一般会生成在catkin_ws目录下，名称为rawMap.txt.每行意义：
经、纬度,x,y,heading
*/


struct DGPS
{
    double utc_second;
    double latitude;
    double longitude;
    double heading;
    double X;
    double Y;
    int satNum;
    int satNum_vice;
    int status_main;  //主站
    int status_vice;  //从站
};

int hex2dec(char ch)
{
    if ('0' <= ch && ch <= '9')
        return ch - '0';
    if ('A' <= ch && ch <= 'F')
        return ch - 'A' + 10;
    return -1;
}

double str2double(const string &str)
{
    istringstream is(str);
    double num;
    is >> num;
    return num;
}


double *coordinateTransfer(double lat,double lon)
{
    // 根据经纬度转成 X, Y坐标系。使用右手系：x+为正东方，y+为正北方，角度为x+逆时针

    //李兆基零点位置
    // double lat0 = 39.99738253818907*PI/180;
    // double lon0 =  116.32848306278056*PI/180;
    
    //美院零点位置
    double lat0 = 39.998907*PI/180;
    double lon0 =  116.329551*PI/180;

    lat = lat*PI/180;
    lon = lon*PI/180;

    double R = 6378245;

    double *point=new double[2];
    point[0] = R *cos(lat0) * (lon-lon0);
    point[1] = R * (lat - lat0); 
    return point;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_port");

    ros::NodeHandle nh;
    ros::Publisher pub_gps = nh.advertise<nav_msgs::Odometry>("gps", 10);
    ros::Publisher pub_gpchc= nh.advertise<std_msgs::String>("gpchc", 10);

    // ros::param 
    // standard serial read
    serial::Serial sp;  
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setTimeout(to);

    try
    {
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    if (sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    // output file stream
    ofstream fsRoad;

    if (SAVE_ROAD_FLAG)
    {
        try
        {
            // append mode
            fsRoad.open("rawMap.txt", ios::app);
        }
        catch (const exception &e)
        {
            ROS_ERROR_STREAM(e.what());
            return -1;
        }

        if (fsRoad.is_open())
        {
            ROS_INFO_STREAM("rawMap.txt is opened.");
        }
        else
        {
            return -1;
        }
    }

    // ros rate match the serial port rate
    ros::Rate rate(20);

    int msglen, tmp, comma[20], commaNum;
    uint8_t checksum, nowsum;
    double dtmp;
    DGPS gps;
    nav_msgs::Odometry gps_msg;

    while (ros::ok())
    {
        if (sp.available())
        {
            std_msgs::String msg;
            msg.data = sp.read(sp.available());
            msglen = msg.data.size();

            if (msg.data[0] == '$')
            {
                tmp = hex2dec(msg.data[msglen - 4]);
                if (tmp == -1)
                    continue;
                checksum = tmp;

                tmp = hex2dec(msg.data[msglen - 3]);
                if (tmp == -1)
                    continue;
                checksum = checksum * 16 + tmp;

                nowsum = 0;
                for (int i = 1; i < msglen - 5; i++)
                    nowsum ^= msg.data[i];

                if (nowsum != checksum)
                {
                    // ROS_ERROR_STREAM("Checksum Failed! nowsum = " << nowsum << ", checksum = " << checksum);
                    // continue;
                }

                string header = msg.data.substr(1, 5);

                commaNum = 0;
                for (int i = 6; i < msglen - 5; i++)  // get positions of all commas
                {
                    if (msg.data[i] == ',')
                        comma[commaNum++] = i;
                }

             
                if (header == "GPCHC")
                {
                    pub_gpchc.publish(msg);

                    if (commaNum != 23  )
                    {
                        ROS_INFO_STREAM("error 23");
                        cout << "error commaNum = " << commaNum << endl;
                        continue;
                    }


                    // UTC
                    dtmp = str2double(msg.data.substr(comma[1] + 1, comma[2] - comma[1] - 1));
                    gps.utc_second =
                        ((int16_t)(dtmp / 10000) + 8) * 3600 + ((int16_t)(dtmp / 100)) % 100 * 60 + fmod(dtmp, 100);

                    // longitude
                    dtmp = str2double(msg.data.substr(comma[12] + 1, comma[13] - comma[12] - 1));
                    gps.longitude = dtmp;
                    cout << "longitude = " << gps.longitude << endl;

                    // latitude
                    dtmp = str2double(msg.data.substr(comma[11] + 1, comma[12] - comma[11] - 1));
                    gps.latitude = dtmp;
                    cout << "latitude = " << gps.latitude << endl;

                    // YAW
                    dtmp = str2double(msg.data.substr(comma[2] + 1, comma[3] - comma[2] - 1));
                    gps.heading =  90 + dtmp;
                    gps_msg.twist.twist.angular.z = (90 + dtmp);
                    ROS_INFO_STREAM("YAW = " << gps_msg.twist.twist.angular.z);
                    // cout << "Yaw = " << gps.heading << endl;

                    // X velocity：东向速度
                    dtmp = str2double(msg.data.substr(comma[14] + 1, comma[15] - comma[14] - 1));
                    gps_msg.twist.twist.linear.x = dtmp;
                    ROS_INFO_STREAM("X velocity = " << dtmp);

                    // Y velocity：北向速度
                    dtmp = str2double(msg.data.substr(comma[15] + 1, comma[16] - comma[15] - 1));
                    gps_msg.twist.twist.linear.y = dtmp;
                    ROS_INFO_STREAM("Y velocity = " << dtmp);

                    // 坐标转换
                    gps.X  = coordinateTransfer(gps.latitude,gps.longitude)[0];
                    gps.Y  = coordinateTransfer(gps.latitude,gps.longitude)[1];

                    gps_msg.pose.pose.position.x = gps.X;
                    gps_msg.pose.pose.position.y = gps.Y;
                    ROS_INFO_STREAM("X = " <<  gps.X);
                    ROS_INFO_STREAM("Y = " <<  gps.Y);
                    // cout << "Y = " << gps.Y << endl;

                    pub_gps.publish(gps_msg);

                    /*save RoadMap*/
                    if (SAVE_ROAD_FLAG)
                    {
                        fsRoad << setprecision(15) <<" "<< gps.longitude<<" " <<gps.latitude<<" "<< gps.X << " " << gps.Y << " " << gps.heading << " " << endl;
                    }
                }
                
            }
        }

        rate.sleep();
    }

    if (fsRoad.is_open())
        fsRoad.close();

    sp.close();

    return 0;
}
