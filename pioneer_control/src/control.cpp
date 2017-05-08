#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "pioneer_control/ControlGoPickUpProductAction.h"
#include "pioneer_control/ControlGoDeliverProductAction.h"

#include "actionlib/client/terminal_state.h"
#include "actionlib/client/simple_action_client.h"
//#include "pioneer_control/NavigationExecutePathAction.h"
#include "pioneer_control/NavigationDriveToAction.h"
#include "pioneer_control/ObjectManipulationPickUpAction.h"
#include "pioneer_control/ObjectManipulationReleaseAction.h"


//typedef actionlib::SimpleActionClient<pioneer_control::NavigationExecutePathAction> NavigationExecutePathClient;
typedef actionlib::SimpleActionClient<pioneer_control::NavigationDriveToAction> NavigationDriveToClient;
typedef actionlib::SimpleActionClient<pioneer_control::ObjectManipulationPickUpAction> ObjectManipulationPickUpClient;
typedef actionlib::SimpleActionClient<pioneer_control::ObjectManipulationReleaseAction> ObjectManipulationReleaseAction;

typedef actionlib::SimpleActionServer<pioneer_control::ControlGoPickUpProductAction> ControlGoPickUpProduct;
typedef actionlib::SimpleActionServer<pioneer_control::ControlGoDeliverProductAction> ControlGoDeliverProduct;
class Control
{
	public:
		Control(ros::NodeHandle n);
	private:
		ros::NodeHandle node;
		NavigationDriveToClient driveToClient;
		pioneer_control::NavigationDriveToGoal driveToGoal;
		pioneer_control::Vec2i32 msg;

		ObjectManipulationPickUpClient pickUpClient;
		pioneer_control::ObjectManipulationPickUpGoal pickUpGoal;

		ObjectManipulationReleaseAction releaseClient;
		pioneer_control::ObjectManipulationReleaseGoal releaseGoal;



		//ros::Subscriber goPickUp;
		//ros::Subscriber goDeliver;
		ControlGoPickUpProduct goPickUpProductServer;
		pioneer_control::ControlGoPickUpProductFeedback goPickUpProductFeedback;
		pioneer_control::ControlGoPickUpProductResult goPickUpProductResult;
		void goPickUpProductAction(const pioneer_control::ControlGoPickUpProductGoalConstPtr& goal);

		ControlGoDeliverProduct goDeliverProductServer;
		pioneer_control::ControlGoDeliverProductFeedback goDeliverProductFeedback;
		pioneer_control::ControlGoDeliverProductResult goDeliverProductResult;
		void goDeliverProductAction(const pioneer_control::ControlGoDeliverProductGoalConstPtr& goal);
};

Control::Control(ros::NodeHandle n)
		: goPickUpProductServer(node, "go_pick_up_product", boost::bind(&Control::goPickUpProductAction, this, _1), false),
		goDeliverProductServer(node, "go_deliver_product", boost::bind(&Control::goDeliverProductAction, this, _1), false),
		driveToClient("drive_to", true),
		pickUpClient("pick_up", true),
		releaseClient("release", true)
{
	node = n;
	goPickUpProductServer.start();
	goDeliverProductServer.start();
}

void Control::goPickUpProductAction(const pioneer_control::ControlGoPickUpProductGoalConstPtr& goal)
{
	driveToGoal.pos = goal->pAPos;
	driveToClient.waitForServer();
	driveToClient.sendGoal(driveToGoal);
	//TODO: subscribe and pass on feedback/2
	driveToClient.waitForResult();
	if(driveToClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
	{
		goPickUpProductResult.status = false;
		goPickUpProductServer.setAborted(goPickUpProductResult);
		ROS_INFO("Something wrong with reaching PA, aborting...");
		return;
	}


	pickUpClient.waitForServer();
	pickUpClient.sendGoal(pickUpGoal);
	pickUpClient.waitForResult();
	if(pickUpClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
	{
		goPickUpProductResult.status = false;
		goPickUpProductServer.setAborted(goPickUpProductResult);
		ROS_INFO("Something wrong with picking up product, aborting...");
		return;
	}

	goPickUpProductResult.status = true;
	goPickUpProductServer.setSucceeded(goPickUpProductResult);
}

void Control::goDeliverProductAction(const pioneer_control::ControlGoDeliverProductGoalConstPtr& goal)
{
	driveToGoal.pos = goal->pAPos;
	ROS_INFO("Driving to deliver pos: (%d, %d).",driveToGoal.pos.x, driveToGoal.pos.y);
	driveToClient.waitForServer();
	driveToClient.sendGoal(driveToGoal);
	//TODO: subscribe and pass on feedback/2
	driveToClient.waitForResult();
	if(driveToClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
	{
		goDeliverProductResult.status = false;
		goDeliverProductServer.setAborted(goDeliverProductResult);
		ROS_INFO("Something wrong with reaching PA, aborting...");
		return;
	}


	releaseClient.waitForServer();
	releaseClient.sendGoal(releaseGoal);
	releaseClient.waitForResult();
	if(releaseClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
	{
		goDeliverProductResult.status = false;
		goDeliverProductServer.setAborted(goDeliverProductResult);
		ROS_INFO("Something wrong with picking up product, aborting...");
		return;
	}

	goDeliverProductResult.status = true;
	goDeliverProductServer.setSucceeded(goDeliverProductResult);
	//TODO: action release product
	//TODO: subscribe and pass on 1/2 + feedback/2
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle node;
	Control control(node);
	
	ros::spin();
	return 0;
}
