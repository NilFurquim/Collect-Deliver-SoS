#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#define DRIVE_ACTUATOR_COMMAND_TOPIC "diff_drive/command"

class DriveActuatorAPI
{
	public:
		DriveActuatorAPI(ros::NodeHandle n);
		void setDrive(double linear, double angular);
	private:
		void clearMessage();
		ros::Publisher drive_pub;
		geometry_msgs::Twist drive_msg;

};

DriveActuatorAPI::DriveActuatorAPI(ros::NodeHandle node)
{
	drive_pub = node.advertise<geometry_msgs::Twist>("diff_drive/command", 100);
	clearMessage();
}

void DriveActuatorAPI::clearMessage()
{
	drive_msg.angular.x = 0;
	drive_msg.angular.y = 0;
	drive_msg.angular.z = 0;
	drive_msg.linear.x = 0;
	drive_msg.linear.y = 0;
	drive_msg.linear.z = 0;
}

void DriveActuatorAPI::setDrive(double linear, double angular)
{
	drive_msg.linear.x = linear;
	drive_msg.angular.z = angular;
	drive_pub.publish(drive_msg);
}
