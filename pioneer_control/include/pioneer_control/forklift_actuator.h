#ifndef _FORKLIFTACTUATOR_H_
#define _FORKLIFTACTUATOR_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include "pioneer_control/ForkliftLift.h"
#include "pioneer_control/ForkliftLower.h"
#include "pioneer_control/ForkliftMoveTo.h"

#define FORKLIFT_LOWER_SERV "ForkliftLower"
#define FORKLIFT_LIFT_SERV "ForkliftLift"
#define FORKLIFT_MOVETO_SERV "ForkliftMoveTo"
#define FORKLIFT_MIN_HEIGHT (-0.1)
#define FORKLIFT_MAX_HEIGHT (0.19)
#define FORKLIFT_ERROR_MIN (-1)
#define FORKLIFT_ERROR_MAX (-2)
#define FORKLIFT_SUCCESS (0)

class ForkLiftActuator
{
	public:
	bool liftServ(pioneer_control::ForkliftLift::Request &req, pioneer_control::ForkliftLift::Response &res);
	bool lowerServ(pioneer_control::ForkliftLower::Request &req, pioneer_control::ForkliftLower::Response &res);
	bool moveToServ(pioneer_control::ForkliftMoveTo::Request &req, pioneer_control::ForkliftMoveTo::Response &res);
	void lowerServ(const std_msgs::Float64::ConstPtr& msg);
	void moveToServ(const std_msgs::Float64::ConstPtr& msg);
	int lift(float amount);
	int lower(float amount);
	void stop();
	int moveTo(float position);
	control_msgs::JointControllerState report();
	ForkLiftActuator(ros::NodeHandle n);

	private:
	ros::ServiceServer lift_service;
	ros::ServiceServer lower_service;
	ros::ServiceServer moveTo_service;
	ros::NodeHandle node;
	float maxHeight;
	float minHeight;
	control_msgs::JointControllerState state;
	float command;
	std_msgs::Float64 command_msg;

	ros::Publisher commander;
	ros::Subscriber subscriber;

	float fork_position_interpolation(float t);
};
#endif
