#include "pioneer_control/forklift_actuator.h"
bool ForkLiftActuator::liftServ(pioneer_control::ForkliftLift::Request &req, pioneer_control::ForkliftLift::Response &res)
{res.status = lift(req.amount);}

bool ForkLiftActuator::lowerServ(pioneer_control::ForkliftLower::Request &req, pioneer_control::ForkliftLower::Response &res)
{res.status = lower(req.amount);}

bool ForkLiftActuator::moveToServ(pioneer_control::ForkliftMoveTo::Request &req, pioneer_control::ForkliftMoveTo::Response &res)
{res.status = moveTo(req.amount);}

ForkLiftActuator::ForkLiftActuator(ros::NodeHandle n)
{
	node = n;
	liftService = n.advertiseService(FORKLIFT_LIFT_SERV, &ForkLiftActuator::liftServ, this);
	lowerService = n.advertiseService(FORKLIFT_LOWER_SERV, &ForkLiftActuator::lowerServ, this);
	moveToService = n.advertiseService(FORKLIFT_MOVETO_SERV, &ForkLiftActuator::moveToServ, this);
	commander = node.advertise<std_msgs::Float64>("forklift_controller/command", 0);
	minHeight = FORKLIFT_MIN_HEIGHT;
	maxHeight = FORKLIFT_MAX_HEIGHT;
};

int ForkLiftActuator::lift(double amount)
{
	double goalPosition = currentPosition + amount;
	return moveTo(goalPosition);
}

int ForkLiftActuator::lower(double amount)
{
	double goalPosition = currentPosition - amount;
	return moveTo(goalPosition);
}

void ForkLiftActuator::stop()
{	ROS_INFO("ForkLiftActuator::stop() not implemented");	}

#define ABS(var) ((var>0)?var:-var)
int ForkLiftActuator::moveTo(double goalPosition)
{
	int ret = FORKLIFT_SUCCESS;
	if (goalPosition >= maxHeight) {
		goalPosition = maxHeight;
		ret = FORKLIFT_ERROR_MAX;
	} else if (goalPosition <= minHeight) {
		goalPosition = minHeight;
		ret = FORKLIFT_ERROR_MIN;
	}
	double initialPosition = currentPosition;
	double t = 0;
	double increment = (0.001*(currentPosition - goalPosition))/
				(FORKLIFT_MAX_HEIGHT - FORKLIFT_MIN_HEIGHT);
	increment = ABS(increment);
	if(increment == 0) t = 1;
	while(t < 1)
	{
		currentPosition = quadraticInterpolation(initialPosition, t, goalPosition);
		commMsg.data = currentPosition;
		commander.publish(commMsg);
		t += increment;
		ros::Duration(0.003).sleep();
		//std::cout << "t: " << t << std::endl;
	}
	t = 1;
	currentPosition = quadraticInterpolation(initialPosition, t, goalPosition);
	commMsg.data = currentPosition;
	commander.publish(commMsg);
	std::cout << "t: " << t << std::endl;
	return ret;
}

control_msgs::JointControllerState report()
{	ROS_INFO("ForkLiftActuator::report() not implemented");	}

double ForkLiftActuator::quadraticInterpolation(double a, double t, double b)
{	return (1 - t * t) * a + t * t * b;}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "forklift_actuator");
	ros::NodeHandle node;
	
	ForkLiftActuator forkLiftActuator(node);
	ros::spin();
	return 0;
}
