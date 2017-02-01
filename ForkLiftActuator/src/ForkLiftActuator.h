#ifndef _FORKLIFTACTUATOR_H_
#define _FORKLIFTACTUATOR_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

class ForkLiftActuator
{
	public:
	void liftCallback(const std_msgs::Float64::ConstPtr& msg);
	void lowerCallback(const std_msgs::Float64::ConstPtr& msg);
	void moveToCallback(const std_msgs::Float64::ConstPtr& msg);
	void lift(float amount);
	void lower(float amount);
	void stop();
	void moveTo(float position);
	control_msgs::JointControllerState report();
	ForkLiftActuator(ros::NodeHandle* n);

	private:
	ros::NodeHandle* node;
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
