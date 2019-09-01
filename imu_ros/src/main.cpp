#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */
#include <geometry_msgs/Quaternion.h>
serial::Serial ser;
# define M_PI           3.14159265358979323846  /* pi */
using namespace std;

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<std_msgs::Float64>("yaw_data", 10);
    std_msgs::Float64 yaw_data;
    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }	
    float yaw=0;

    while(ros::ok()){
        if(ser.available()){
            string result;
            result = ser.readline();
            int n = result.length(); 
            char data[n + 1]; 
            strcpy(data, result.c_str()); 
            if(data[0]=='Y')  {
		yaw=(float)atoi(&data[1]);
		if (yaw == 0 || yaw == 360) {yaw_data.data = 0;ROS_INFO("Yaw: %d", 0);}
		else if(yaw<180) {yaw_data.data=(-yaw) * (M_PI/180);ROS_INFO("Yaw: %f", -yaw);}
		else  {yaw_data.data=(360-yaw) * (M_PI/180); ROS_INFO("Yaw: %f", (360-yaw));}
			
		}
	    
            imu_pub.publish(yaw_data);
        }
        ros::spinOnce();
    }
}

