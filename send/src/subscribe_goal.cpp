#include <ros/ros.h>
#include <publish_goals/goals.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <tf/tf.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
# define M_PI           3.14159265358979323846  /* pi */
# define power_scale	0.25	/* [2-4] * power_scale = [0,1] */

/* 
 - Weight:
   0 = obstacle so theres probably no 0's 
   1 = clean  
   2 - 4 is dirt level 
*/

//     /goal

float current_x, current_y, current_theta;
int in = 0; // so for spinOnce
int go = 0;
int goal_index = 0; // for array above
int num_goal = 3;
double angleToGoal;
geometry_msgs::Twist cmd;
float angular_speed = 0.1; // rotation speed
float linear_speed = 0.1; // linear speed
float xCallBack;
float yCallBack;
int sizeCallBack = -1;
std::vector<float> x;	
std::vector<float> y;
std::vector<float> weight;
bool done = false;

/* efficient turning */
void turning()
{
	cmd.linear.x = 0;	
	if (angleToGoal >= 0 && current_theta >= 0) // positive/on the right
	{
		if (angleToGoal > current_theta)
		{
			cmd.angular.z =  angular_speed;	
		}

		else
		{
			cmd.angular.z = -angular_speed;
		}
	}
	else if (angleToGoal <= 0 && current_theta <= 0) // negative/on the left
	{
		if (angleToGoal > current_theta)
		{
			cmd.angular.z =  angular_speed;	
		}
		else
		{
			cmd.angular.z = - angular_speed;	
		}
	}
	else	// one is positive and one is negative
	{
		if (angleToGoal < 0)	// angleToGoal is negative && current is positive
		{
			float bot = (angleToGoal - M_PI) + (M_PI - current_theta);
			float top = (0 - angleToGoal) + current_theta;
			if (top > bot)
			{
				cmd.angular.z = - angular_speed;	// 		
			}
			else 
			{
				cmd.angular.z = angular_speed;
			}
		}
		else 	// angleToGoal is + && current is -
		{
			float bot = (current_theta - M_PI) + (M_PI - angleToGoal);
			float top = (0 - current_theta) + angleToGoal;
			if (top > bot)
			{
				cmd.angular.z = angular_speed;
			}
			else 
			{
				cmd.angular.z = - angular_speed;
			}
		}
	}
}

void goalCallBack(const publish_goals::goals::ConstPtr& msg)
{
	for (int i = 0; i < msg->size; i++)
	{
		x.push_back(msg->x[i]);
		y.push_back(msg->y[i]);
		weight.push_back(msg->weight[i]);
	}
	done = true;
	//ROS_INFO("goal_index: %d", goal_index);
	//ROS_INFO("x: %f", msg->x[goal_index]);
	//weightCallBack = msg->weight[goal_index]
	//power = weightCallBack * ;
	sizeCallBack = msg->size;
}

void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;    
   	current_theta = tf::getYaw(msg->pose.pose.orientation);
	//ROS_INFO("Current theta:%f", current_theta * 180/3.14);
}


/* efficient turning */

int main(int argc, char** argv )
 {
    ros::init(argc, argv, "send");
    ros::NodeHandle nh_;
        
    ros::Subscriber goal_sub = nh_.subscribe("/goal", 50, &goalCallBack);
    ros::Subscriber pose_sub = nh_.subscribe("robot_0/amcl_pose", 50, &poseCallBack);    
    ros::Publisher dirt_motor_pub = nh_.advertise<std_msgs::Float32>("robot_0/dirt_motor",50);
    ros::Publisher cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 50);

    std_msgs::Float32 power;
    double distance;   
    ros::Rate loop_rate(100);  
	
	while(ros::ok())
	{			
		if (done)	// since it takes a little bit before getting the information
		{	
			if (goal_index == sizeCallBack)
			{
				ros::shutdown();
			}
			/* set motor power*/		
		 	power.data = weight.at(goal_index) * 0.25; 
		 
	   	 	double x_ = x.at(goal_index) - current_x;
           	 	double y_ = y.at(goal_index) - current_y;
		 	angleToGoal = atan2(y_,x_);
           	 	distance = sqrt(pow(y_, 2) + pow(x_, 2)); // distance formula
		

			double minus = angleToGoal - current_theta;
			/* Absolute Value */
			if (minus < 0)
			{
				minus = (-1) * minus;
			}
			/* if angle isn't lined up */
			if (minus > 0.1) 
			{
				turning();
			}
                	/* once robot's angle lines up, it will go straight the next time it loops */
            		else
			{   	
				distance = sqrt(pow(y_, 2) + pow(x_, 2)); // distance formula
				if (distance < 0.09) // if goal is reach
           	 		{
					goal_index = goal_index + 1;	// next goal
		    			cmd.angular.z = 0.0;
           	     			cmd.linear.x = 0.0; // stop
					
				}
				else
				{            			
				cmd.angular.z = 0.0;	// stop turning
			        cmd.linear.x = linear_speed; // go straight  
				}
			 }
		
	    		cmd_vel_pub.publish(cmd);
	    		dirt_motor_pub.publish(power);             
		}		
		ros::spinOnce();
		loop_rate.sleep();
	}/* end of while*/
    
    ros::shutdown();
    return 0;
 }
