#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "pioneer_control/image_processing.h"
#include "pioneer_control/NavigationDriveToAction.h"
#include "pioneer_control/NavigationExecutePathAction.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/terminal_state.h"
#include "pioneer_control/DriveActuatorAPI.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32.h"
#include <deque>
#include "pioneer_control/Vec2.h"
#include "pioneer_control/PoseGrid.h"
#include "pioneer_control/PathPlanningDefinePath.h"
#define LOCALIZATION_TOPIC "map_position"

#define ABS(var) ((var<0)? -var : var)
#define PRINT_M for(int i = 0; i < height; i++)\
	{\
		printf("| ");\
		for(int j = 0; j < width; j++)\
		{\
			printf("%d ", processedImg->data[i*height + j]);\
		}\
		printf("|\n");\
	}\
	printf("\n")
//Functions:
//driveTo(directions: Direction)
//executePath(path: Path)
//stop()

//Handle:
//CameraSensor:(?)
//ImageProcessing: handleProcessedImage

//Use:
//Localization:
//DriveActuator:
//MapInformation:
//PathPlanning:

typedef actionlib::SimpleActionServer<pioneer_control::NavigationDriveToAction> NavigationDriveToServer;
typedef actionlib::SimpleActionServer<pioneer_control::NavigationExecutePathAction> NavigationExecutePathServer;

class Navigation
{
	public:
		Navigation(ros::NodeHandle n, Vec2i pos, Vec2i dir);
	private:
		enum Action {Action_stop, Action_turn_around, Action_turn_right, Action_turn_left, Action_go_straight, Action_follow_line};
		static const Vec2i dirs[4];
		ros::NodeHandle node;
		DriveActuatorAPI driveApi;

		Action action;
		Vec2i currentLocalization;
		Vec2i currentDirection;

		ros::Subscriber processedImageSub, localizationSub;
		bool isNavigationOn, isCrossing;
		double identifyCrossingThreshold, stopTurningThreshold;
		std::deque<Action> actions;
		std::deque<Action> executedActions;
		bool waitLocalization;
		
		Action convertToAction(Vec2i prev, Vec2i next);
		void startNavigation();
		void stopNavigation();
		void testExecutePath();
		bool checkPath();
		void handleLocalizationChange(const pioneer_control::PoseGrid localization);
		void handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processed);
		bool hasLineOnTheSides(const std_msgs::Int16MultiArrayConstPtr& processed, int height, int width);
		Action nextPathAction();

		ros::ServiceClient definePathClient;
		pioneer_control::PathPlanningDefinePath definePathService;

		//Vec2i msg2Vec2i(pioneer_control::Vec2i32 v);

		void driveToAction(const pioneer_control::NavigationDriveToGoalConstPtr& goal);
		NavigationExecutePathServer navExecutePathServer;
		pioneer_control::NavigationExecutePathFeedback navExecutePathFeedback;
		pioneer_control::NavigationExecutePathResult navExecutePathResult;

		void executePathAction(const pioneer_control::NavigationExecutePathGoalConstPtr& goal);
		NavigationDriveToServer navDriveToServer;
		pioneer_control::NavigationDriveToFeedback navDriveToFeedback;
		pioneer_control::NavigationDriveToResult navDriveToResult;
};

//Vec2i Navigation::msg2Vec2i(pioneer_control::Vec2i32 v)
//{
//	return Vec2i(v.x, v.y);
//}

Navigation::Action Navigation::nextPathAction()
{
	if(actions.empty())
		return Action_stop;

	Action front = actions.front();
	executedActions.push_back(front);
	actions.pop_front();
	return front;
}

Navigation::Action Navigation::convertToAction(Vec2i prev, Vec2i next)
{
	//ROS_INFO("\n(%d, %d) -> (%d, %d)", prev.x, prev.y, next.x, next.y);
	if(next == Vec2i(-1, -1)) Action_stop;

	int v = prev.x * next.y - prev.y * next.x;
	//ROS_INFO("\nv = %d", v);
	if(v == 0)
	{
		if(prev == next){
			return Action_go_straight;
		} else {
			return Action_turn_around;
		}
	}
	if(v < 0)
		return Action_turn_left;
	else
		return Action_turn_right;
}

double linearInterpolation(double min, double t, double max)
{return (1 - t) * min + t * max;}

double quadraticInterpolation(double min, double t, double max)
{return (1 - t*ABS(t)) * min + t*ABS(t) * max;}

double cubicInterpolation(double min, double t, double max)
{return (1 - t*t*t) * min + t*t*t * max;}

void Navigation::startNavigation()
{ isNavigationOn = true; action = Action_follow_line;}
void Navigation::stopNavigation()
{ driveApi.setDrive(0, 0); isNavigationOn = false; }

void Navigation::handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processedImg)
{
	if(!isNavigationOn) return;
	
	int height = processedImg->layout.dim[0].size;
	int width = processedImg->layout.dim[1].size;
	int offset = (height-1)*width;
	const short int *firstLine = &processedImg->data[0];
	const short int *lastLine = &processedImg->data[offset];

	int firstLineSum, lastLineSum, total_sum;
	float angularRegulator, linearRegulator;
	float firstLineAvg, lastLineAvg;

	total_sum = 0;
	for(int i = 0; i < height * width; i++)
		total_sum += processedImg->data[i];


	//stop if no guide line is identified
	if(total_sum == 0){
		//printf("sum == 0\n");
		driveApi.setDrive(0, 0);
		return;
	}

	//crossing reached
	if(total_sum >= identifyCrossingThreshold*(height*width) && isCrossing == false)
	{
		printf("next action\n");
		action = nextPathAction();
		isCrossing = true;
		switch (action) {
		case Action_stop: printf("stop\n"); break;
		case Action_turn_right: printf("turn_right\n"); break;
		case Action_turn_left: printf("turn_left\n"); break;
		case Action_go_straight: printf("go_straight\n"); break;
		case Action_follow_line: printf("follow_line\n"); break;
		default: printf("default\n"); break;
		}
	}

	switch (action) {
	case Action_stop:
		driveApi.setDrive(0, 0);
		isCrossing = false;
		break;
	case Action_turn_right:
		driveApi.setDrive(0.055, 0.4);
		if(total_sum <= 28) {
			action = Action_follow_line;
			isCrossing = false;
		}
		break;
	case Action_turn_left:
		driveApi.setDrive(0.055, -0.4);
		if(total_sum <= 28) {
			action = Action_follow_line;
			isCrossing = false;
			printf("change\n");
		}
		break;
		actions.size();

	case Action_go_straight:
	case Action_follow_line:
		//printf("firstLineAvg = %f\n", firstLineAvg);
		//printf("lastLineAvg = %f\n", lastLineAvg);


		/*
		printf("avg=%f\n",avg);
		printf("total_sum=%d\n",total_sum);
		printf("norm=%f\n",normalized_avg);
		*/

		lastLineSum = firstLineSum = 0;
		lastLineAvg = firstLineAvg = 0;
		for(int i = 0; i < width; i++)
		{
			lastLineSum += lastLine[i];
			firstLineSum += firstLine[i];
			firstLineAvg += firstLine[i]*(i+1-(width+1)/2.0);
			lastLineAvg += lastLine[i]*(i+1-(width+1)/2.0);
		}
	
		firstLineAvg /= firstLineSum;
		lastLineAvg /= lastLineSum;
		firstLineAvg /= width/2.0;
		lastLineAvg /= width/2.0;

		//"normalized" and centered
		if(firstLineSum == 0){
			angularRegulator = lastLineAvg;
		}else{
			angularRegulator = firstLineAvg * ABS(lastLineAvg);
		}

		if(lastLineSum == 0) angularRegulator = 1;
		linearRegulator = 1 - ABS(angularRegulator);
		driveApi.setDrive(quadraticInterpolation(0.04,	linearRegulator, 0.35),
				  quadraticInterpolation(0.00, angularRegulator, 1.7));
		if(total_sum <= 28) {
			action = Action_follow_line;
			isCrossing = false;
		}
		break;
	default:
		action = Action_stop;
		break;
	}
}

void Navigation::driveToAction(const pioneer_control::NavigationDriveToGoalConstPtr& goalmsg)
{
	//ROS_INFO("\nLOCAL (%d, %d)", local.x, local.y);
	definePathService.request.origin.x = currentLocalization.x;
	definePathService.request.origin.y = currentLocalization.y;
	Vec2i goal(goalmsg->pos.x,goalmsg->pos.y);

	definePathService.request.goal = goalmsg->pos;
	if(!definePathClient.call(definePathService))
	{
		ROS_INFO("Not Able to get path from path planning service");
		navDriveToFeedback.progress = 0;
		navDriveToServer.publishFeedback(navDriveToFeedback);
		navDriveToResult.status = false;
		navDriveToServer.setAborted(navDriveToResult);
		return;
	}

	typedef std::vector<pioneer_control::Vec2i32> vectorVec2i32msg;
	vectorVec2i32msg nodePath = definePathService.response.path;
	
	if(nodePath.size() == 0)
	{
		ROS_INFO("No path to goal");
		navDriveToFeedback.progress = 0;
		navDriveToServer.publishFeedback(navDriveToFeedback);
		navDriveToResult.status = false;
		navDriveToServer.setAborted(navDriveToResult);
		return;
	}

	if(nodePath.size() == 1)
	{
		ROS_INFO("Origin == Goal");
		navDriveToFeedback.progress = 1;
		navDriveToServer.publishFeedback(navDriveToFeedback);
		navDriveToResult.status = true;
		navDriveToServer.setSucceeded(navDriveToResult);
		return;
	}

	//convert path to directions
	std::vector<Vec2i> directions;
	for(vectorVec2i32msg::iterator it = nodePath.begin()+1; it < nodePath.end(); it++)
	{
		directions.push_back(Vec2i((*(it)).x, (*(it)).y) -
					Vec2i((*(it-1)).x, (*(it-1)).y)); 
		//printf("(%d, %d) ", directions.back().x, directions.back().y);
	}
	
	actions.clear();
	//convert directions to actions
	ROS_INFO("\n" 
		"current direction (%d, %d)\n"
		"next direction (%d, %d)", 
		currentDirection.x, currentDirection.y, 
		directions[0].x, directions[0].y);

	actions.push_back(convertToAction(currentDirection, directions[0]));

	for(int i = 1; i < directions.size(); i++)
	{
		actions.push_back(convertToAction(directions[i-1], directions[i]));
	}
	actions.push_back(Action_stop);

	navDriveToFeedback.progress = 0.0;
	navDriveToServer.publishFeedback(navDriveToFeedback);

	int actionsTotal = actions.size();

	startNavigation();
	pioneer_control::Vec2i32ConstPtr waitLocal;
	waitLocal = ros::topic::waitForMessage<pioneer_control::Vec2i32>(LOCALIZATION_TOPIC, node);

	navDriveToFeedback.progress = executedActions.size()*1.0/actionsTotal;
	navDriveToServer.publishFeedback(navDriveToFeedback);

	while(currentLocalization != goal)
	{
		//wait for localization 
		//waitForLocalizationChange();
		if(!checkPath())
		{
			//wrong way - cancel executePath and set not succeded
		}
		waitLocal = ros::topic::waitForMessage<pioneer_control::Vec2i32>(LOCALIZATION_TOPIC, node);
		navDriveToFeedback.progress = executedActions.size()*1.0/actionsTotal;
		navDriveToServer.publishFeedback(navDriveToFeedback);
	}

	if(actions.size() == 1 && actions[0] == Action_stop)
	{
		ROS_INFO("Almost done, waiting to get to node.");
		while(!actions.empty());
		navDriveToFeedback.progress = 1;
		navDriveToServer.publishFeedback(navDriveToFeedback);
		navDriveToResult.status = true;
		navDriveToServer.setSucceeded(navDriveToResult);
		stopNavigation();
		ROS_INFO("DriveTo succeeded.!");
		return;
	}

	navDriveToResult.status = false;
	navDriveToServer.setAborted(navDriveToResult);
	stopNavigation();
	ROS_INFO("DriveTo aborted.");

}

void Navigation::handleLocalizationChange(const pioneer_control::PoseGrid localization)
{
	currentLocalization.x = localization.pos.x;
	currentLocalization.y = localization.pos.y;
}

bool Navigation::checkPath()
{
	//if actions.size() < distance(local, goal) return false;
	//if local is not in path return false;
	return true;
}

void Navigation::executePathAction(const pioneer_control::NavigationExecutePathGoalConstPtr& goal)
{
	if(goal->directions.size() == 0)
	{
		navExecutePathFeedback.progress = 1;
		navExecutePathServer.publishFeedback(navExecutePathFeedback);
		navExecutePathResult.status = true;
		navExecutePathServer.setSucceeded(navExecutePathResult);
		return;
	}
	actions.clear();
	actions.push_back(convertToAction(
				currentDirection, 
				Vec2i(goal->directions[0].x, goal->directions[0].y)));

	for(int i = 1; i < goal->directions.size(); i++)
	{
		actions.push_back(convertToAction(
					Vec2i(goal->directions[i-1].x,goal->directions[i-1].y),
					Vec2i(goal->directions[i].x, goal->directions[i].y)));
	}
	actions.push_back(Action_stop);

	navExecutePathFeedback.progress = 0.0;
	navExecutePathServer.publishFeedback(navExecutePathFeedback);
	int actionsTotal = actions.size();

	startNavigation();
	pioneer_control::Vec2i32ConstPtr waitLocal;
	while(!actions.empty())
	{
		//wait for localization 
		/*waitLocal = */ros::topic::waitForMessage<pioneer_control::Vec2i32>(LOCALIZATION_TOPIC, node);
		if(!checkPath())
		{
			//wrong way - cancel executePath and set not succeded
		}
		navExecutePathFeedback.progress = executedActions.size()*1.0/actionsTotal;
		navExecutePathServer.publishFeedback(navExecutePathFeedback);
	}
	navExecutePathResult.status = true;
	navExecutePathServer.setSucceeded(navExecutePathResult);
	stopNavigation();

}

Navigation::Navigation(ros::NodeHandle n, Vec2i pos, Vec2i dir) : 
	navDriveToServer(node, "drive_to", boost::bind(&Navigation::driveToAction, this, _1), false),
	navExecutePathServer(node, "execute_path", boost::bind(&Navigation::executePathAction, this, _1), false),
	driveApi(n)
{
	node = n;
	processedImageSub = node.subscribe<std_msgs::Int16MultiArray>(IMAGE_PROCESSED_TOPIC, 1,  &Navigation::handleProcessedImage, this);
	localizationSub = node.subscribe<pioneer_control::PoseGrid>(LOCALIZATION_TOPIC, 10, &Navigation::handleLocalizationChange, this);
	definePathClient = node.serviceClient<pioneer_control::PathPlanningDefinePath>("define_path");

	navDriveToServer.start();
	//percentage covered in black
	identifyCrossingThreshold = 0.6;
	stopTurningThreshold = 0.44;

	currentLocalization = pos;
	currentDirection = dir;
	//TODO: call init localization
	action = Action_follow_line;
	isCrossing = false;
	isNavigationOn = false;
	waitLocalization = false;

	navExecutePathServer.start();
	navDriveToServer.start();
	ROS_INFO("Servers started!");
	//testExecutePath();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation");
	ros::NodeHandle node;
	
	Navigation navigation(node, Vec2i(0, 0), Vec2i(0, 1));
	ros::spin();
	return 0;
}
