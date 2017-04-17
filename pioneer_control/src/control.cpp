#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "pioneer_control/ControlGoPickUpProductAction.h"
#include "pioneer_control/ControlGoDeliverProductAction.h"

#include "actionlib/client/terminal_state.h"
#include "actionlib/client/simple_action_client.h"
//#include "pioneer_control/NavigationExecutePathAction.h"
#include "pioneer_control/NavigationDriveToAction.h"


//typedef actionlib::SimpleActionClient<pioneer_control::NavigationExecutePathAction> NavigationExecutePathClient;
typedef actionlib::SimpleActionClient<pioneer_control::NavigationDriveToAction> NavigationDriveToClient;

typedef actionlib::SimpleActionServer<pioneer_control::ControlGoPickUpProductAction> ControlGoPickUpProduct;
typedef actionlib::SimpleActionServer<pioneer_control::ControlGoDeliverProductAction> ControlGoDeliverProduct;
class Control
{
	public:
		Control(ros::NodeHandle node);
	private:
		NavigationDriveToClient driveToClient;
		pioneer_control::NavigationDriveToGoal driveToGoal;
		pioneer_control::Vec2i32 msg;

		ControlGoPickUpProduct goPickUpProductServer;
		pioneer_control::ControlGoPickUpProductFeedback goPickUpProductFeedback;
		pioneer_control::ControlGoPickUpProductResult goPickUpProductResult;
		void goPickUpProductAction(const pioneer_control::ControlGoPickUpProductGoalConstPtr& goal);

		ControlGoDeliverProduct goDeliverProductServer;
		pioneer_control::ControlGoDeliverProductFeedback goDeliverProductFeedback;
		pioneer_control::ControlGoDeliverProductResult goDeliverProductResult;
		void goDeliverProductAction(const pioneer_control::ControlGoDeliverProductGoalConstPtr& goal);
};

Control::Control(ros::NodeHandle node)
		: goPickUpProductServer(node, "pick_up_product", boost::bind(&Control::goPickUpProductAction, this, _1), false),
		goDeliverProductServer(node, "deliver_product", boost::bind(&Control::goDeliverProductAction, this, _1), false),
		 driveToClient("drive_to", true)
{
}

void Control::goPickUpProductAction(const pioneer_control::ControlGoPickUpProductGoalConstPtr& goal)
{
	driveToGoal.pos = goal->pAPos;
	driveToClient.sendGoal(driveToGoal);
	//TODO: subscribe and pass on feedback/2
	driveToClient.waitForResult();

	//TODO: action pick up product
	//TODO: subscribe and pass on 1/2 + feedback/2
}

void Control::goDeliverProductAction(const pioneer_control::ControlGoDeliverProductGoalConstPtr& goal)
{
	driveToGoal.pos = goal->pAPos;
	driveToClient.sendGoal(driveToGoal);
	//TODO: subscribe and pass on feedback/2
	driveToClient.waitForResult();

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
