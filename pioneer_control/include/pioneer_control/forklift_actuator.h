#ifndef _FORKLIFTACTUATOR_H_
#define _FORKLIFTACTUATOR_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include "pioneer_control/ForkliftLift.h"
#include "pioneer_control/ForkliftLower.h"
#include "pioneer_control/ForkliftMoveTo.h"

#define FORKLIFT_LOWER_SERV "forklift_lower"
#define FORKLIFT_LIFT_SERV "forklift_lift"
#define FORKLIFT_MOVETO_SERV "forklift_move_to"
#define FORKLIFT_MIN_HEIGHT (-0.13)
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
	int lift(double amount);
	int lower(double amount);
	void stop();
	int moveTo(double position);
	control_msgs::JointControllerState report();
	ForkLiftActuator(ros::NodeHandle n);

	private:
	ros::ServiceServer liftService;
	ros::ServiceServer lowerService;
	ros::ServiceServer moveToService;
	ros::NodeHandle node;
	double maxHeight;
	double minHeight;
	control_msgs::JointControllerState state;
	double currentPosition;
	std_msgs::Float64 commMsg;

	ros::Publisher commander;
	ros::Subscriber subscriber;

	double quadraticInterpolation(double a, double t, double b);
};
#endif
