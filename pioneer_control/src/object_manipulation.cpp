#include "ros/ros.h"

//Msgs
#include "pioneer_control/RangeArray.h"
#include "nav_msgs/Odometry.h"

//Services
#include "pioneer_control/ForkLiftMoveTo.h"

//Actions
#include "actionlib/server/simple_action_server.h"
#include "pioneer_control/ObjectManipulationPickUpAction.h"
#include "pioneer_control/ObjectManipulationReleaseAction.h"

//Utils
#include "pioneer_control/DriveActuatorAPI.h"
#include "pioneer_control/Vec2.h"

#define FORKLIFT_NAVPOS 1.00
#define FORKLIFT_PICKPOS -0.12
#define FORKLIFT_CARRYPOS 0.13
#define PI 3.14159265358979323846

typedef actionlib::SimpleActionServer<pioneer_control::ObjectManipulationPickUpAction> ObjectManipulationPickUpServer;
typedef actionlib::SimpleActionServer<pioneer_control::ObjectManipulationReleaseAction> ObjectManipulationReleaseServer;

class ObjectManipulation
{
	public:
		ObjectManipulation(ros::NodeHandle n);
	private:
		enum Action {Action_move_pick_up, Action_move_release, Action_turn_around, 
			Action_turn_around_rotate, Action_turn_around_go_back, 
			Action_go_forward, Action_stop, Action_wait};

		enum Status {Status_has_product, Status_no_product};

		ros::NodeHandle node;
		Action action;
		Status status;
		DriveActuatorAPI driveApi;

		double totalAngle, totalLinear;
		Vec2d prevTotalOffset, totalOffset;
		double prevLinearSpeed, prevAngularSpeed;
		ros::Time prevTime;
		bool isTrackingMovement;
		void startMovementTracking();
		void stopMovementTracking();

		bool isMoving;
		void startMovePickUpAction();
		void startMoveReleaseAction();
		void startTurnAroundAction();
		void startGoForward();
		bool isActionDone();

		ros::ServiceClient moveForkClient;
		pioneer_control::ForkLiftMoveTo moveForkService;

		ros::Subscriber odometrySub;
		void handleOdometry(const nav_msgs::OdometryConstPtr& odom);

		ros::Subscriber frontSensorsSub;
		void handleFrontSensorsChange(const pioneer_control::RangeArray ranges);

		ObjectManipulationPickUpServer pickUpServer;
		pioneer_control::ObjectManipulationPickUpFeedback pickUpFeedback;
		pioneer_control::ObjectManipulationPickUpResult pickUpResult;
		void pickUpAction(const pioneer_control::ObjectManipulationPickUpGoalConstPtr& goal);

		ObjectManipulationReleaseServer releaseServer;
		pioneer_control::ObjectManipulationReleaseFeedback releaseFeedback;
		pioneer_control::ObjectManipulationReleaseResult releaseResult;
		void releaseAction(const pioneer_control::ObjectManipulationReleaseGoalConstPtr& goal);
};


ObjectManipulation::ObjectManipulation(ros::NodeHandle n)
	: node(n), driveApi(n),
	pickUpServer(node, "pick_up", boost::bind(&ObjectManipulation::pickUpAction, this, _1), false),
	releaseServer("release", boost::bind(&ObjectManipulation::releaseAction, this, _1), false)
{
	frontSensorsSub = node.subscribe<pioneer_control::RangeArray>("front_sensors_ranges", 1, &ObjectManipulation::handleFrontSensorsChange, this);
	odometrySub = node.subscribe<nav_msgs::Odometry>("diff_drive/odom", 1, &ObjectManipulation::handleOdometry, this);
	moveForkClient = node.serviceClient<pioneer_control::ForkLiftMoveTo>("forklift_move_to");
	moveForkService.request.amount = FORKLIFT_NAVPOS;
	if(!moveForkClient.call(moveForkService))
	{
		ROS_INFO("Couldn't move fork, aborting...");
		ros::shutdown();
		return;
	}
	action = Action_move_pick_up;
	status = Status_no_product;
	isMoving = false;
	isTrackingMovement = false;
	prevTime = ros::Time::now();
	totalOffset = Vec2d(0.0, 0.0);
	totalAngle = 0;
	pickUpServer.start();
	releaseServer.start();
	ROS_INFO("Object manipulation servers started!");
}

double linearInterpolation(double min, double t, double max)
{return (1 - t) * min + t * max;}

#define ABS(var) (((var)>0)?(var):-(var))
double quadraticInterpolation(double min, double t, double max)
{return (1 - t*ABS(t)) * min + t*ABS(t) * max;}

double cubicInterpolation(double min, double t, double max)
{return (1 - t*t*t) * min + t*t*t * max;}

void ObjectManipulation::startMovementTracking()
{ 
	totalOffset = Vec2d(0, 0);
	totalAngle = 0;
	prevLinearSpeed = 0;
	prevAngularSpeed = 0;
	isTrackingMovement = true;
}

void ObjectManipulation::stopMovementTracking()
{ isTrackingMovement = false; }

void ObjectManipulation::startMovePickUpAction()
{
	action = Action_move_pick_up;
	isMoving = true;
}

void ObjectManipulation::startMoveReleaseAction()
{
	action = Action_move_release;
	isMoving = true;
}

void ObjectManipulation::startTurnAroundAction()
{
	action = Action_turn_around;
	isMoving = true;
}

void ObjectManipulation::startGoForward()
{
	action = Action_go_forward;
	isMoving = true;
}

bool ObjectManipulation::isActionDone()
{ return action == Action_wait; }

void ObjectManipulation::handleOdometry(const nav_msgs::OdometryConstPtr& odom)
{
	if(!isTrackingMovement)
	{
		prevTime = ros::Time::now();
		return;
	}
	double dt = (ros::Time::now() - prevTime).toSec();
	prevTotalOffset = totalOffset;

	Vec2d local = Vec2d(sin(prevAngularSpeed*dt)*prevLinearSpeed*dt, 
			cos(prevAngularSpeed*dt)*prevLinearSpeed*dt);
	local.rotate(totalAngle);
	totalAngle += prevAngularSpeed*dt;
	totalLinear += prevLinearSpeed*dt;
	totalOffset += local;
	prevLinearSpeed = odom->twist.twist.linear.x;
	prevAngularSpeed = odom->twist.twist.angular.z;
	prevTime = ros::Time::now();
}

void ObjectManipulation::handleFrontSensorsChange(pioneer_control::RangeArray ranges)
{
	if(!isMoving)
	{
		prevTime = ros::Time::now();
		return;
	}
	int sensorsAmount = ranges.ranges.size();
	double directionSum = 0;
	double min = ranges.maxRange;
	double idir;

	for(int i = 0; i < sensorsAmount; i++)
	{
		idir = (i-(sensorsAmount-1)/2.0)*(ranges.maxRange - ranges.ranges[i])/ranges.maxRange;

		directionSum += idir;
		if(ranges.ranges[i] < min)
		{
			min = ranges.ranges[i];
		}
	}
	directionSum /= (sensorsAmount-1)/2;
	directionSum = -directionSum;

	double linearRegulator = (1 - directionSum)*min;
	double linearSpeed, angularSpeed;

	Vec2d off;
	Vec2d prevOff;
	double lin;
	double ang;
	switch(action)
	{
		case Action_move_pick_up:
			driveApi.setDrive(linearInterpolation(0.00, linearRegulator, 0.2),
				       	  linearInterpolation(0.00, directionSum, 0.3));
			if(directionSum < 0.01 && min < 0.05) 
			{
				ROS_INFO("STOP");
				action = Action_stop;
			}
			break;
		case Action_move_release:
			driveApi.setDrive(linearInterpolation(0.00, -linearRegulator, 0.2),
				       	  linearInterpolation(0.00, directionSum, 0.3));
			if(min > 0.25)
			{
				ROS_INFO("STOP");
				action = Action_stop;
			}
			break;
		case Action_turn_around:
			//off.x = ABS(prevTotalOffset.x) - ABS(totalOffset.x);
			//off.y = ABS(prevTotalOffset.y) - ABS(totalOffset.y);
			////prevOff.x = ABS(prevTotalOffset.x);
			////prevOff.y = ABS(prevTotalOffset.y);
			//
			//lin = 0.1;
			//ang = 0;
			////off = totalOffset - prevTotalOffset;
			//if(ABS(off.x + off.y) > ABS(off.x - off.y))
			//{
			//	lin = 0.05;
			//	ang = off.y / ABS(off.y);
			//}
			//else if(off.x > 0 && off.y > 0)
			//{
			//	lin = 0.001;
			//	ang = off.y / ABS(off.y);
			//}

			//printf("(%lf, %lf)\n", totalOffset.x, totalOffset.y);
			//driveApi.setDrive(lin, ang);
			//if(totalOffset.x < 0.1 && totalOffset.y < 0.1)
			//{
			//	action = Action_turn_around_rotate;
			//}
			if(totalAngle < 0)
			{
				if(totalAngle < -PI) ang = -0.3;
				else ang = 0.3;
			}
			else
			{
				if(totalAngle > PI) ang = 0.3;
				else ang = -0.3;
			}

			driveApi.setDrive(0, ang);
			//std::cout << ABS(totalAngle - PI) << std::endl;
			if(ABS(totalAngle - PI) < 0.4 || ABS(totalAngle + PI) < 0.4)
			{
				action = Action_turn_around_go_back;
			}
			break;
		case Action_turn_around_rotate:
			action = Action_stop;
			break;
		case Action_turn_around_go_back:
			action = Action_stop;
			break;
		case Action_go_forward:
			driveApi.setDrive(0.2, 0);
			if(totalLinear > 0.4)
			{
				action = Action_stop;
			}
			break;
		case Action_stop:
			driveApi.setDrive(0, 0);
			action = Action_wait;
			break;
		case Action_wait:
			isMoving = false;
			break;
		default:
			break;
	}
	prevTime = ros::Time::now();
}

void ObjectManipulation::pickUpAction(const pioneer_control::ObjectManipulationPickUpGoalConstPtr& goal)
{
	if(status == Status_has_product)
	{
		ROS_INFO("Already handling a product, must release first, aborting...");
		pickUpServer.setAborted();
		return;
	}
	moveForkService.request.amount = FORKLIFT_PICKPOS;
	if(!moveForkClient.call(moveForkService))
	{
		ROS_INFO("Couldn't move fork, aborting...");
		pickUpServer.setAborted();
		return;
	}

	startMovementTracking();
	startMovePickUpAction();
	while(!isActionDone());
	
	moveForkService.request.amount = FORKLIFT_CARRYPOS;
	if(!moveForkClient.call(moveForkService))
	{
		ROS_INFO("Couldn't move fork, aborting...");
		pickUpServer.setAborted();
		return;
	}
	startTurnAroundAction();
	while(!isActionDone());
	stopMovementTracking();

	pickUpServer.setSucceeded();
	status = Status_has_product;
	return;

}

void ObjectManipulation::releaseAction(const pioneer_control::ObjectManipulationReleaseGoalConstPtr& goal)
{
	if(status == Status_no_product)
	{
		ROS_INFO("No product to release.");
		releaseServer.setAborted();
		return;
	}

	startMovementTracking();
	startGoForward();
	while(!isActionDone());
	moveForkService.request.amount = FORKLIFT_PICKPOS;
	if(!moveForkClient.call(moveForkService))
	{
		ROS_INFO("Couldn't move fork, aborting...");
		releaseServer.setAborted();
		return;
	}

	startMoveReleaseAction();
	while(!isActionDone());
	moveForkService.request.amount = FORKLIFT_NAVPOS;
	if(!moveForkClient.call(moveForkService))
	{
		ROS_INFO("Couldn't move fork, aborting...");
		releaseServer.setAborted();
		return;
	}

	startTurnAroundAction();
	while(!isActionDone());
	stopMovementTracking();

	releaseServer.setSucceeded();
	status = Status_no_product;
	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_manipulation");
	ros::NodeHandle node;
	ObjectManipulation objectManipulation(node);
	ros::spin();
	return 0;
}
