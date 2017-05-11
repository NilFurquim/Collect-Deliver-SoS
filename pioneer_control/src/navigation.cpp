#include "ros/ros.h"

//Services
#include "pioneer_control/PathPlanningDefinePath.h"
#include "pioneer_control/LocalizationInit.h"
#include "pioneer_control/MapInformationUpdateMap.h"

//Actions
#include "actionlib/server/simple_action_server.h"
#include "pioneer_control/NavigationDriveToAction.h"
typedef actionlib::SimpleActionServer<pioneer_control::NavigationDriveToAction> NavigationDriveToServer;
#include "pioneer_control/NavigationExecutePathAction.h"
typedef actionlib::SimpleActionServer<pioneer_control::NavigationExecutePathAction> NavigationExecutePathServer;

//Msgs
#include "std_msgs/Int16MultiArray.h"
#include "pioneer_control/PoseGrid.h"
#include "pioneer_control/UpdateMapMsg.h"

//Utils
#include "pioneer_control/DriveActuatorAPI.h"
#include "pioneer_control/Vec2.h"
#include <deque>
typedef std::vector<pioneer_control::Vec2i32> vectorVec2i32msg;

#define LOCALIZATION_TOPIC "localization"

#define ABS(var) (((var)<0)? -(var) : (var))
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


class Navigation
{
	public:
		Navigation(ros::NodeHandle n, int id, Vec2i pos, Vec2i dir);
	private:
		ros::NodeHandle node;
		DriveActuatorAPI driveApi;
		int id;

		enum Action {Action_leave_crossing, Action_wait, Action_stop, 
			Action_turn_around, Action_turn_around2, Action_turn_around3, 
			Action_turn_right, Action_turn_left, Action_go_straight, 
			Action_follow_line};
		Action action;

		Vec2i currentLocalization;
		Vec2i currentDirection;

		bool isNavigationOn, isCrossing;
		double identifyCrossingThreshold, stopTurningThreshold;
		std::deque<Action> actions;
		std::deque<Action> executedActions;
		std::deque<Vec2i> directions;
		std::deque<Vec2i> path;
		bool isPathValid, isNodeValid;
		bool isPositionValid;
		Action convertToAction(Vec2i prev, Vec2i next);
		void startNavigation();
		void stopNavigation();
		void testExecutePath();
		bool checkPath();
		bool nextPathAction();

		ros::Subscriber localizationSub;
		pioneer_control::LocalizationInit localizationInitService;
		ros::ServiceClient localizationInitClient;
		bool waitLocalization;
		void handleLocalizationChange(const pioneer_control::PoseGrid localization);

		ros::ServiceClient updateMapClient;
		pioneer_control::MapInformationUpdateMap updateMapService;

		ros::Subscriber positionChangeSub;
		void handleOtherRobotsLocalizationChange(const pioneer_control::UpdateMapMsg foreignLocalization);
		
		ros::Subscriber processedImageSub;
		void handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processed);

		ros::ServiceClient definePathClient;
		pioneer_control::PathPlanningDefinePath definePathService;

		//Vec2i msg2Vec2i(pioneer_control::Vec2i32 v);

		bool driveToAction(const pioneer_control::NavigationDriveToGoalConstPtr& goal);
		//NavigationExecutePathServer navExecutePathServer;
		//pioneer_control::NavigationExecutePathFeedback navExecutePathFeedback;
		//pioneer_control::NavigationExecutePathResult navExecutePathResult;

		//void executePathAction(const pioneer_control::NavigationExecutePathGoalConstPtr& goal);
		NavigationDriveToServer navDriveToServer;
		pioneer_control::NavigationDriveToFeedback navDriveToFeedback;
		pioneer_control::NavigationDriveToResult navDriveToResult;
};

//Vec2i Navigation::msg2Vec2i(pioneer_control::Vec2i32 v)
//{
//	return Vec2i(v.x, v.y);
//}

bool Navigation::nextPathAction()
{
	if(actions.empty())
	{
		ROS_INFO("actions empty.");
		return false;
	}

	action = actions.front();
	actions.pop_front();
	executedActions.push_back(action);
	return true;
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
{ isNavigationOn = true; action = Action_follow_line; }
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
	double angularRegulator, linearRegulator;
	double firstLineAvg, lastLineAvg;

	total_sum = 0;
	for(int i = 0; i < height * width; i++)
		total_sum += processedImg->data[i];


	//stop if no guide line is identified
	if(action == Action_follow_line && total_sum == 0){
		driveApi.setDrive(0, 0.1);
		return;
	}
	if(total_sum >= identifyCrossingThreshold*(height*width))
		{ isCrossing = true; }
	else 
		{ isCrossing = false; }

	//crossing reached
	if(action == Action_follow_line && isCrossing)
	{
		action = Action_stop;
		//switch (action) {
		//case Action_stop: printf("stop\n"); break;
		//case Action_turn_right: printf("turn_right\n"); break;
		//case Action_turn_left: printf("turn_left\n"); break;
		//case Action_go_straight: printf("go_straight\n"); break;
		//case Action_follow_line: printf("follow_line\n"); break;
		//case Action_turn_around: printf("turn_around\n"); break;
		//case Action_turn_around2: printf("turn_around_2\n"); break;
		//default: printf("default\n"); break;
		//}
	}

	switch (action) {
	case Action_stop:
		printf("Action_stop\n");
		driveApi.setDrive(0, 0);
		action = Action_wait;
		break;
	case Action_turn_right:
		//printf("Action_turn_right\n");
		driveApi.setDrive(0.055, 0.4);
		action = Action_leave_crossing;
		break;
	case Action_turn_left:
		//printf("Action_turn_left\n");
		driveApi.setDrive(0.055, -0.4);
		action = Action_leave_crossing;
		break;
	case Action_go_straight:
		//printf("Action_go_straight\n");
	case Action_follow_line:
		lastLineSum = firstLineSum = 0;
		lastLineAvg = firstLineAvg = 0;
		for(int i = 0; i < width; i++)
		{
			lastLineSum += lastLine[i];
			firstLineSum += firstLine[i];
			firstLineAvg += firstLine[i]*(i-(width-1)/2.0);
			lastLineAvg += lastLine[i]*(i-(width-1)/2.0);
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

		if(lastLineSum == 0) angularRegulator = 0;
		linearRegulator = 1 - ABS(angularRegulator);
		driveApi.setDrive(cubicInterpolation(0.03, linearRegulator, 0.17),
				  quadraticInterpolation(0.00, angularRegulator, 1.0));
		if(action == Action_go_straight && total_sum <= 28) {
			action = Action_follow_line;
		}
		break;
	case Action_turn_around:
		//printf("Action_turn_around\n");
		driveApi.setDrive(0, 0.5);
		action = Action_turn_around2;
		break;
	case Action_turn_around2:
		if(total_sum <= 10)
		{
			driveApi.setDrive(0, 0.3);
			action = Action_turn_around3;
		}
		break;
	case Action_turn_around3:
		if(total_sum >= 28)
		{
			action = Action_follow_line;
		}
		break;
	case Action_leave_crossing:
		if(total_sum <= 20) {
			action = Action_follow_line;
		}
		break;
	case Action_wait:
		break;
	default:
		driveApi.setDrive(0, 0);
		break;
	}
}

bool Navigation::driveToAction(const pioneer_control::NavigationDriveToGoalConstPtr& goalmsg)
{
	bool isDriveToDone = false;
	do {
		if(!ros::ok())
		{
			navDriveToResult.status = false;
			navDriveToServer.setAborted(navDriveToResult);
			stopNavigation();
			ROS_INFO("Ros not ok, DriveTo aborting...");
			return false;
		}
		definePathService.request.origin.x = currentLocalization.x;
		definePathService.request.origin.y = currentLocalization.y;
		Vec2i goal(goalmsg->pos.x,goalmsg->pos.y);

		definePathService.request.goal = goalmsg->pos;
		if(!definePathClient.call(definePathService))
		{
			//ROS_INFO("Not Able to get path from path planning service");
			//ROS_INFO("Not Able to get path from path planning service, trying again in 2 seconds");
			//stopNavigation();
			//navDriveToFeedback.progress = 0;
			//navDriveToServer.publishFeedback(navDriveToFeedback);
			//navDriveToResult.status = false;
			//navDriveToServer.setAborted(navDriveToResult);
			ros::Duration(2).sleep(); //Trying again!
			continue;
			return false;
		}

		vectorVec2i32msg nodePath = definePathService.response.path;
		isPathValid = true;
		
		if(nodePath.size() == 0)
		{
			//ROS_INFO("No path to goal, aborting...");
			//ROS_INFO("No path to goal, trying again in 2 seconds...");
			//stopNavigation();
			//navDriveToFeedback.progress = 0;
			//navDriveToServer.publishFeedback(navDriveToFeedback);
			//navDriveToResult.status = false;
			//navDriveToServer.setAborted(navDriveToResult);
			ros::Duration(2).sleep(); //Trying again!
			continue;
			return false;
		}

		if(nodePath.size() == 1)
		{
			//Maybe it won't turn around...
			ROS_INFO("Origin == Goal, success!");
			startNavigation();
			while(action != Action_wait);
			stopNavigation();
			navDriveToFeedback.progress = 1;
			navDriveToServer.publishFeedback(navDriveToFeedback);
			navDriveToResult.status = true;
			navDriveToServer.setSucceeded(navDriveToResult);
			return true;
		}

		
		printf("Path:\n");
		path.clear();
		for(vectorVec2i32msg::iterator it = nodePath.begin(); it < nodePath.end(); it++)
		{
			path.push_back(Vec2i((*it).x, (*it).y));
			printf("\t(%d, %d)\n", (*it).x, (*it).y);
		}

		//convert path to directions
		directions.clear();
		for(std::deque<Vec2i>::iterator it = path.begin()+1; it < path.end(); it++)
		{
			directions.push_back((*it) - *(it-1));
		}
		
		actions.clear();
		executedActions.clear();
		//convert directions to actions
		//ROS_INFO("\n" 
		//	"(%d, %d) -> (%d, %d)", 
		//	currentDirection.x, currentDirection.y, 
		//	directions[0].x, directions[0].y);

		actions.push_back(convertToAction(currentDirection, directions[0]));

		for(int i = 1; i < directions.size(); i++)
		{
			actions.push_back(convertToAction(directions[i-1], directions[i]));
		}

		navDriveToFeedback.progress = 0.0;
		navDriveToServer.publishFeedback(navDriveToFeedback);

		int actionsTotal = actions.size();
		startNavigation();
		while(action != Action_wait);
		nextPathAction();

		ros::Duration waitTime(10);
		pioneer_control::PoseGridConstPtr waitLocal;
		navDriveToFeedback.progress = executedActions.size()*1.0/actionsTotal;
		navDriveToServer.publishFeedback(navDriveToFeedback);
		//while(true) {
			if(!checkPath())
			{
				isDriveToDone = false;
				if(!isNodeValid)
				{
					while(!isNodeValid);
					action = Action_follow_line;
				}
				while(action != Action_wait);
				break;
			}
			path.pop_front();
			directions.pop_front();

			//updateMapService.request.info.id = id*1000 + 1;
			//if(!path.empty())
			//{
			//	updateMapService.request.info.pose.pos.x = path.front().x;
			//	updateMapService.request.info.pose.pos.y = path.front().y;
			//	updateMapService.request.info.pose.dir.x = directions.front().x;
			//	updateMapService.request.info.pose.dir.y = directions.front().y;
			//}
			//if(!updateMapClient.call(updateMapService))
			//{
			//	ROS_INFO("Couldn't update map for next pos!");
			//}

			if(currentLocalization == goal)
			{
				if(!isCrossing)
				{
					startNavigation();
				}
				while(action != Action_wait);
				isDriveToDone = true;
				break;
			}

			do {
				waitLocal = ros::topic::waitForMessage<pioneer_control::PoseGrid>(LOCALIZATION_TOPIC, waitTime);
				//TODO: Check how many times while ran
			} while(waitLocal == NULL);
			if(!isPositionValid)
			{
				ROS_INFO("%d: Position invalid.", id);
				//stopNavigation();
				//waitTime.sleep();
			}

			while(action != Action_wait);
			nextPathAction();

			navDriveToFeedback.progress = executedActions.size()*1.0/actionsTotal;
			navDriveToServer.publishFeedback(navDriveToFeedback);
		//}
		
	} while(!isDriveToDone);


}

void Navigation::handleLocalizationChange(const pioneer_control::PoseGrid localization)
{
	currentLocalization.x = localization.pos.x;
	currentLocalization.y = localization.pos.y;
	currentDirection.x = localization.dir.x;
	currentDirection.y = localization.dir.y;
	if(!isNavigationOn || action == Action_turn_around || action == Action_turn_around2) return;
	updateMapService.request.info.id = id;
	updateMapService.request.info.pose.pos.x = localization.pos.x;
	updateMapService.request.info.pose.pos.y = localization.pos.y;
	updateMapService.request.info.pose.dir.x = localization.dir.x;
	updateMapService.request.info.pose.dir.y = localization.dir.y;
	if(!updateMapClient.call(updateMapService))
	{
		ROS_INFO("Couldn't update map!");
		if(updateMapService.response.status == false)
		{
			ROS_INFO("Current robot's position is invalid!");
			isPositionValid = false;
		}
	}
}

void Navigation::handleOtherRobotsLocalizationChange(const pioneer_control::UpdateMapMsg foreignLocalization)
{
	//Check if new location affects path
	int foreignId = foreignLocalization.id;
	if(foreignId == id) return;
	isPathValid = true;
	isNodeValid = true;

	if(path[0].x == foreignLocalization.pose.pos.x && path[0].y == foreignLocalization.pose.pos.y)
	{
		if(foreignId < id)
		{
			isNodeValid = false;
			isPathValid = false;
			action = Action_stop;
			return;
		}
	}

	for(int i = 1; i < 3; i++)
	{
		//If location is in path and is not the same direction then path is invalid
		if(path[i].x == foreignLocalization.pose.pos.x && path[i].y == foreignLocalization.pose.pos.y
		   // && !(directions[i].x == foreignLocalization.dir.x && directions[i].y == foreignLocalization.dir.y))
			)
		{
			ROS_INFO("Path invalidated!");
			isPathValid = false;
			return;
		}

	}
	isPathValid = true;
}

bool Navigation::checkPath()
{
	//if actions.size() < distance(local, goal) return false;
	//if local is not in path return false;
	//if(currentLocalization != path.front())
	//{
	//	ROS_INFO("%d: Current is not actual path!\nIs (%d, %d) but expected (%d, %d).", id, currentLocalization.x, currentLocalization.y, path.front().x, path.front().y);
	//	return false;
	//}
	if(!isPathValid)
	{
		ROS_INFO("Some robot in the way!");
		return false;
	}

	return true;
}

/*void Navigation::executePathAction(const pioneer_control::NavigationExecutePathGoalConstPtr& goal)
{
	if(goal->directions.size() == 0)
	{
		navExecutePathFeedback.progress = 1;
		navExecutePathServer.publishFeedback(navExecutePathFeedback);
		navExecutePathResult.status = true;
		navExecutePathServer.setSucceeded(navExecutePathResult);
		return;
	}

	directions.clear();
	for(vectorVec2i32msg::const_iterator it = goal->directions.begin(); it != goal->directions.end(); it++)
	{
		directions.push_back(Vec2i(it->x, it->y));

	}
	actions.clear();
	actions.push_back(convertToAction(currentDirection, directions[0]));

	for(std::deque<Vec2i>::iterator it = directions.begin()+1; it != directions.end(); it++)
	{
		actions.push_back(convertToAction(*(it-1), *it));
	}
	actions.push_back(Action_stop);

	navExecutePathFeedback.progress = 0.0;
	navExecutePathServer.publishFeedback(navExecutePathFeedback);
	int actionsTotal = actions.size();

	startNavigation();
	pioneer_control::PoseGridConstPtr waitLocal;
	while(!actions.empty())
	{
		//wait for localization 
		//waitLocal =
		  ros::topic::waitForMessage<pioneer_control::PoseGrid>(LOCALIZATION_TOPIC, node);
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

}*/

Navigation::Navigation(ros::NodeHandle n, int id, Vec2i pos, Vec2i dir) : 
	navDriveToServer(node, "drive_to", boost::bind(&Navigation::driveToAction, this, _1), false),
	//navExecutePathServer(node, "execute_path", boost::bind(&Navigation::executePathAction, this, _1), false),
	driveApi(n), id(id), node(n)
{
	localizationInitClient = node.serviceClient<pioneer_control::LocalizationInit>("localization/init");
	localizationInitService.request.pos.pos.x = pos.x;
	localizationInitService.request.pos.pos.y = pos.y;
	localizationInitService.request.pos.dir.x = dir.x;
	localizationInitService.request.pos.dir.y = dir.y;
	if(!localizationInitClient.call(localizationInitService))
	{
		ROS_INFO("Couldn't call localizationInit, shutting down...");
		ros::shutdown();
	}
	else if(!localizationInitService.response.status)
	{
		
		ROS_INFO("Position not valid, shutting down...");
		ros::shutdown();
	}
	processedImageSub = node.subscribe<std_msgs::Int16MultiArray>("image_processing/processed", 1,  &Navigation::handleProcessedImage, this);
	localizationSub = node.subscribe<pioneer_control::PoseGrid>(LOCALIZATION_TOPIC, 10, &Navigation::handleLocalizationChange, this);
	positionChangeSub = node.subscribe<pioneer_control::UpdateMapMsg>("/pos_change", 5, &Navigation::handleOtherRobotsLocalizationChange, this);
	definePathClient = node.serviceClient<pioneer_control::PathPlanningDefinePath>("define_path");
	updateMapClient = node.serviceClient<pioneer_control::MapInformationUpdateMap>("/update_map");
	//percentage covered in black
	identifyCrossingThreshold = 0.6;
	stopTurningThreshold = 0.44;

	currentLocalization = pos;
	currentDirection = dir;
	action = Action_follow_line;
	isCrossing = false;
	isNavigationOn = false;
	waitLocalization = false;
	isPathValid = true;
	isNodeValid = true;
	isPositionValid = true;

	//navExecutePathServer.start();
	navDriveToServer.start();
	ROS_INFO("Servers started!");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation");
	ros::NodeHandle node;
	if(argc != 6)
	{
		printf("Navigation: requires 4 arguments.\n"
			"id gridPos.x gridPos.y dir.x dir.y\n");
	}
	
	int id = atoi(argv[1]);
	Vec2i pos = Vec2i(atoi(argv[2]), atoi(argv[3]));
	Vec2i dir = Vec2i(atoi(argv[4]), atoi(argv[5]));
	printf("PASSED PARAMS: %d %d %d %d %d", id, pos.x, pos.y, dir.x, dir.y);
	Navigation navigation(node, id, pos, dir);
	ros::spin();
	return 0;
}
