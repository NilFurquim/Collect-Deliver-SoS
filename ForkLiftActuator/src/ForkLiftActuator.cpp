#include "ForkLiftActuator.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ForkLiftActuator");
	ros::NodeHandle node;
	
	ForkLiftActuator forkLiftActuator(&node);
	ros::Subscriber lift_sub = node.subscribe<std_msgs::Float64>("ForkLiftActuator/lift",1000, &ForkLiftActuator::liftCallback, &forkLiftActuator);
	ros::Subscriber lower_sub = node.subscribe<std_msgs::Float64>("ForkLiftActuator/lower",1000, &ForkLiftActuator::lowerCallback, &forkLiftActuator);
	ros::Subscriber moveTo_sub = node.subscribe<std_msgs::Float64>("ForkLiftActuator/moveTo",1000, &ForkLiftActuator::moveToCallback, &forkLiftActuator);
	ros::spin();
	return 0;
}

void ForkLiftActuator::liftCallback(const std_msgs::Float64::ConstPtr& msg)
{	lift((*msg).data);	}

void ForkLiftActuator::lowerCallback(const std_msgs::Float64::ConstPtr& msg)
{	lower((*msg).data);	}

void ForkLiftActuator::moveToCallback(const std_msgs::Float64::ConstPtr& msg)
{	moveTo((*msg).data);	}

ForkLiftActuator::ForkLiftActuator(ros::NodeHandle* n)
{
	node = n;
	commander = (*node).advertise<std_msgs::Float64>("fork_lift_controller/command",1000);
	minHeight = -0.1;
	maxHeight = 0.13;
};

void ForkLiftActuator::lift(float amount)
{
	if (command + amount >= 1) 
	{
		command = 1;
	} else command += amount;
	command_msg.data = fork_position_interpolation(command);
	commander.publish(command_msg);
}

void ForkLiftActuator::lower(float amount)
{
	if (command - amount <= 0)
	{
		command = 0;
	} else command -= amount;
	command_msg.data = fork_position_interpolation(command);
	commander.publish(command_msg);
}

void ForkLiftActuator::stop()
{	ROS_INFO("ForkLiftActuator::stop() not implemented");	}

void ForkLiftActuator::moveTo(float position)
{
	if (position >= maxHeight) position = maxHeight;
	else
	if (position <= minHeight) position = minHeight;
	command_msg.data = position;

	commander.publish(command_msg);
}

control_msgs::JointControllerState report()
{	ROS_INFO("ForkLiftActuator::report() not implemented");	}

float ForkLiftActuator::fork_position_interpolation(float t)
{	return (1 - t) * minHeight + t * maxHeight;}

