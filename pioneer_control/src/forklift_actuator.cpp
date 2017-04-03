#include "pioneer_control/forklift_actuator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "forklift_actuator");
	ros::NodeHandle node;
	
	ForkLiftActuator forkLiftActuator(node);
	ros::spin();
	return 0;
}
bool ForkLiftActuator::liftServ(pioneer_control::ForkliftLift::Request &req, pioneer_control::ForkliftLift::Response &res)
{res.status = lift(req.percentage);}

bool ForkLiftActuator::lowerServ(pioneer_control::ForkliftLower::Request &req, pioneer_control::ForkliftLower::Response &res)
{res.status = lower(req.percentage);}

bool ForkLiftActuator::moveToServ(pioneer_control::ForkliftMoveTo::Request &req, pioneer_control::ForkliftMoveTo::Response &res)
{res.status = moveTo(req.amount);}

ForkLiftActuator::ForkLiftActuator(ros::NodeHandle n)
{
	node = n;
	liftService = n.advertiseService(FORKLIFT_LIFT_SERV, &ForkLiftActuator::liftServ, this);
	lowerService = n.advertiseService(FORKLIFT_LOWER_SERV, &ForkLiftActuator::lowerServ, this);
	moveToService = n.advertiseService(FORKLIFT_MOVETO_SERV, &ForkLiftActuator::moveToServ, this);
	commander = node.advertise<std_msgs::Float64>("forklift_controller/command",1000);
	minHeight = FORKLIFT_MIN_HEIGHT;
	maxHeight = FORKLIFT_MAX_HEIGHT;
};

int ForkLiftActuator::lift(float amount)
{
	int ret = FORKLIFT_SUCCESS;
	if (command + amount >= 1) 
	{
		ret = FORKLIFT_ERROR_MAX;
		command = 1;
	} else {
		ret = FORKLIFT_SUCCESS;
		command += amount;
	}
	commMsg.data = forkPositionInterpolation(command);
	commander.publish(commMsg);
	return ret;
}

int ForkLiftActuator::lower(float amount)
{
	int ret = FORKLIFT_SUCCESS;
	if (command - amount <= 0)
	{
		ret = FORKLIFT_ERROR_MIN;
		command = 0;
	} else {
		ret = FORKLIFT_SUCCESS;
		command -= amount;
	}
	commMsg.data = forkPositionInterpolation(command);
	commander.publish(commMsg);
	return ret;
}

void ForkLiftActuator::stop()
{	ROS_INFO("ForkLiftActuator::stop() not implemented");	}

int ForkLiftActuator::moveTo(float position)
{
	int ret = FORKLIFT_SUCCESS;
	if (position >= maxHeight) {
		position = maxHeight;
		ret = FORKLIFT_ERROR_MAX;
	} else if (position <= minHeight) {
		position = minHeight;
		ret = FORKLIFT_ERROR_MIN;
	}
	commMsg.data = position;
	commander.publish(commMsg);
	return ret;
}

control_msgs::JointControllerState report()
{	ROS_INFO("ForkLiftActuator::report() not implemented");	}

float ForkLiftActuator::forkPositionInterpolation(float t)
{	return (1 - t) * minHeight + t * maxHeight;}

