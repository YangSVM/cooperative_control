#include <fstream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#define SAVE_ROAD_FLAG false

using namespace std;

struct DGPS
{
    double utc_second;
    double latitude;
    double longitude;
    double heading;
    double X;
    double Y;
    int satNum;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_port");

    ros::NodeHandle nh;
    ros::Publisher pub_gps = nh.advertise<nav_msgs::Odometry>("gps", 10);

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

    ofstream fsRoad;

    if (SAVE_ROAD_FLAG)
    {
        try
        {
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
                    ROS_ERROR_STREAM("Checksum Failed! nowsum = " << nowsum << ", checksum = " << checksum);
                    continue;
                }

                string header = msg.data.substr(1, 5);

                commaNum = 0;
                for (int i = 6; i < msglen - 5; i++)  // get positions of all commas
                {
                    if (msg.data[i] == ',')
                        comma[commaNum++] = i;
                }

                if (header == "GPYBM")
                {
                    if (commaNum != 23)
                        continue;

                    // UTC
                    dtmp = str2double(msg.data.substr(comma[1] + 1, comma[2] - comma[1] - 1));
                    gps.utc_second =
                        ((int16_t)(dtmp / 10000) + 8) * 3600 + ((int16_t)(dtmp / 100)) % 100 * 60 + fmod(dtmp, 100);

                    // longitude
                    dtmp = str2double(msg.data.substr(comma[2] + 1, comma[3] - comma[2] - 1));
                    gps.longitude = dtmp;
                    // cout << "longitude = " << gps.longitude << endl;

                    // latitude
                    dtmp = str2double(msg.data.substr(comma[3] + 1, comma[4] - comma[3] - 1));
                    gps.latitude = dtmp;
                    // cout << "latitude = " << gps.latitude << endl;

                    // YAW
                    dtmp = str2double(msg.data.substr(comma[5] + 1, comma[6] - comma[5] - 1));
                    gps.heading = dtmp;
                    gps_msg.twist.twist.angular.z = dtmp;
                    ROS_INFO_STREAM("YAW = " << dtmp);
                    // cout << "Yaw = " << gps.heading << endl;

                    // X velocity
                    dtmp = str2double(msg.data.substr(comma[7] + 1, comma[8] - comma[7] - 1));
                    gps_msg.twist.twist.linear.x = dtmp;
                    ROS_INFO_STREAM("X velocity = " << dtmp);

                    // Y velocity
                    dtmp = str2double(msg.data.substr(comma[8] + 1, comma[9] - comma[8] - 1));
                    gps_msg.twist.twist.linear.y = dtmp;
                    ROS_INFO_STREAM("Y velocity = " << dtmp);

                    // Gauss X
                    dtmp = str2double(msg.data.substr(comma[11] + 1, comma[12] - comma[11] - 1));
                    gps.X = dtmp;
                    gps_msg.pose.pose.position.x = dtmp;
                    ROS_INFO_STREAM("X = " << dtmp);
                    // cout << "X = " << gps.X << endl;

                    // Gauss Y
                    dtmp = str2double(msg.data.substr(comma[12] + 1, comma[13] - comma[12] - 1));
                    gps.Y = dtmp;
                    gps_msg.pose.pose.position.y = dtmp;
                    ROS_INFO_STREAM("Y = " << dtmp);
                    // cout << "Y = " << gps.Y << endl;

                    // status
                    char ch = msg.data[comma[15] + 1];
                    if (ch < '0' || '5' < ch)
                        continue;
                    gps.status_main = ch - '0';
                    // cout << "status = " << gps.status_main << endl;

                    ch = msg.data[comma[16] + 1];
                    if (ch < '0' || '5' < ch)
                        continue;
                    gps.status_vice = ch - '0';
                    // cout << "status = " << gps.status_vice << endl;

                    // satNum
                    dtmp = str2double(msg.data.substr(comma[17] + 1, comma[18] - comma[17] - 1));
                    gps.satNum = dtmp;
                    // cout << "satnum = " << gps.satNum << endl;

                    pub_gps.publish(gps_msg);

                    /*save RoadMap*/
                    if (SAVE_ROAD_FLAG)
                    {
                        fsRoad << setprecision(15) << 1 << " " << gps.X << " " << gps.Y << " " << gps.heading << " " << 4 << " "
                               << 0 << endl;
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