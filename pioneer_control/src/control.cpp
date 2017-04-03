#include "ros/ros.h"
#include "pioneer_control/NavigationExecutePathAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

enum Action {Action_stop, Action_turn_right, Action_turn_left, Action_go_straight, Action_follow_line};
typedef actionlib::SimpleActionClient<pioneer_control::NavigationExecutePathAction> NavigationExecutePathClient;

class Control
{
	public:
		Control(ros::NodeHandle node);
	private:
};

Control::Control(ros::NodeHandle node)
{
	//goPickUpProductService = node.advertise;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle node;
	NavigationExecutePathClient navExecutePathClient("execute_path", true);
	ROS_INFO("Waiting for server...");
	navExecutePathClient.waitForServer();
	pioneer_control::NavigationExecutePathGoal goal;
	goal.path.push_back((int)Action_go_straight);
	goal.path.push_back((int)Action_go_straight);
	goal.path.push_back((int)Action_turn_left);
	goal.path.push_back((int)Action_turn_left);
	goal.path.push_back((int)Action_turn_right);
	goal.path.push_back((int)Action_go_straight);
	goal.path.push_back((int)Action_turn_right);
	goal.path.push_back((int)Action_go_straight);
	ROS_INFO("Sending goal...");
	navExecutePathClient.sendGoal(goal);
	ROS_INFO("Waiting for Result...");
	navExecutePathClient.waitForResult();
	ROS_INFO("DONE!");
	
	ros::spin();
	return 0;
}
