#include "ros/ros.h"
#include "pioneer_control/NavigationExecutePathAction.h"
#include "pioneer_control/NavigationDriveToAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

typedef actionlib::SimpleActionClient<pioneer_control::NavigationExecutePathAction> NavigationExecutePathClient;
typedef actionlib::SimpleActionClient<pioneer_control::NavigationDriveToAction> NavigationDriveToClient;

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

void executePath()
{
	NavigationExecutePathClient navExecutePathClient("execute_path", true);
	ROS_INFO("Waiting for server...");
	navExecutePathClient.waitForServer();
	pioneer_control::NavigationExecutePathGoal goal;
	pioneer_control::Vec2i32 msg;
	msg.x = 0; msg.y = 1; 
	goal.directions.push_back(msg);
	msg.x = 0; msg.y = 1; 
	goal.directions.push_back(msg);
	msg.x = 1; msg.y = 0; 
	goal.directions.push_back(msg);
	msg.x = 0; msg.y = -1; 
	goal.directions.push_back(msg);
	msg.x = 1; msg.y = 0; 
	goal.directions.push_back(msg);
	msg.x = 0; msg.y = 1; 
	goal.directions.push_back(msg);
	msg.x = 0; msg.y = 1; 
	goal.directions.push_back(msg);
	msg.x = -1; msg.y = 0; 
	goal.directions.push_back(msg);
	ROS_INFO("Sending goal...");
	navExecutePathClient.sendGoal(goal);
	ROS_INFO("Waiting for Result...");
	navExecutePathClient.waitForResult();
	ROS_INFO("DONE!");
}

void driveTo(int x, int y)
{
	NavigationDriveToClient navDriveToClient("drive_to", true);
	ROS_INFO("Waiting for server...");
	navDriveToClient.waitForServer();
	pioneer_control::NavigationDriveToGoal goal;
	pioneer_control::Vec2i32 msg;
	msg.x = x; msg.y = y; 
	goal.pos = msg;
	ROS_INFO("Sending goal...");
	navDriveToClient.sendGoal(goal);
	ROS_INFO("Waiting for Result...");
	navDriveToClient.waitForResult();
	ROS_INFO("DONE!");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle node;
	driveTo(1, 5);
	driveTo(1, 1);
	driveTo(0, 2);
	
	ros::spin();
	return 0;
}
