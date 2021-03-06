#include "ros/ros.h"
#include <iostream>
#include <ncurses.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "pioneer_control/ForkliftLift.h"
#include "pioneer_control/ForkliftLower.h"

#define DRIVE_ROBOT_TOPIC "diff_drive/command"
#define DRIVE_MAX_LINEAR 0.7
#define DRIVE_MAX_ANGULAR 1


int main(int argc, char **argv)
{
	initscr();
	cbreak();
	ros::init(argc, argv, "operation");
	ros::NodeHandle n;
	ros::Rate loop_rate(5);

	ros::Publisher driveRobot_pub;
	ros::Publisher fork_lift;
	ros::Publisher fork_lower;

	geometry_msgs::Twist twist_msg;
	std_msgs::Float64 fork_msg;

	driveRobot_pub = n.advertise<geometry_msgs::Twist>(DRIVE_ROBOT_TOPIC, 0);
	ros::ServiceClient fork_lift_client = n.serviceClient<pioneer_control::ForkliftLift>("ForkliftLift");
	ros::ServiceClient fork_lower_client = n.serviceClient<pioneer_control::ForkliftLower>("ForkliftLower");
	pioneer_control::ForkliftLift fork_lift_srv;
	pioneer_control::ForkliftLower fork_lower_srv;

	twist_msg.linear.x = 0;
	twist_msg.linear.y = 0;
	twist_msg.linear.z = 0;
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = 0;
	fork_lift_srv.request.amount = 0.1;
	fork_lower_srv.request.amount = 0.1;

	char input = 'q';
	while(ros::ok())
	{
		input = getch();
		std::fflush;
		switch (input)
		{
			//forward
			case 'w':
				//ROS_INFO("w");
				twist_msg.linear.x = 0.2;
				driveRobot_pub.publish(twist_msg);
				break;
			//left
			case 'a':
				//ROS_INFO("a");
				twist_msg.angular.z = -0.2;
				driveRobot_pub.publish(twist_msg);
				break;
			//break/backward
			case 's':
				//ROS_INFO("s");
				twist_msg.linear.x = -0.2;
				driveRobot_pub.publish(twist_msg);
				break;
			//right
			case 'd':
				//ROS_INFO("d");
				twist_msg.angular.z = 0.2;
				driveRobot_pub.publish(twist_msg);
				break;
			//break
			case 'l':
				//ROS_INFO("l");
				twist_msg.linear.x = 0;
				twist_msg.angular.z = 0;
				driveRobot_pub.publish(twist_msg);
				break;
			//lower fork
			case 'j':
				fork_lift_client.call(fork_lift_srv);
				break;
			//lift fork
			case 'k':
				fork_lower_client.call(fork_lower_srv);
				break;
			default:
				//ROS_INFO("default");
				twist_msg.linear.x = 0;
				twist_msg.angular.z = 0;
				driveRobot_pub.publish(twist_msg);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
