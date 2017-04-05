#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "pioneer_control/image_processing.h"
#include "pioneer_control/NavigationDriveToAction.h"
#include "pioneer_control/NavigationExecutePathAction.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "pioneer_control/DriveActuatorAPI.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32.h"
#include <deque>
#include "pioneer_control/Vec2.h"
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
typedef actionlib::SimpleActionClient<pioneer_control::NavigationExecutePathAction> NavigationExecutePathClient;

class Navigation
{
	public:
		Navigation(ros::NodeHandle n);
	private:
		enum Action {Action_stop, Action_turn_right, Action_turn_left, Action_go_straight, Action_follow_line};
		ros::NodeHandle node;
		Action action;

		DriveActuatorAPI driveApi;
		ros::Subscriber processedImageSub, localizationSub;
		bool isNavigationOn, isCrossing;
		double identifyCrossingThreshold, stopTurningThreshold;
		std::deque<Action> directions;
		std::deque<Action> executedDirections;
		Vec2i local;
		bool localizationLock;
		
		void testExecutePath();
		void waitForLocalizationChange();
		bool checkPath();
		void handleLocalizationChange(const geometry_msgs::Pose2D localization);
		void handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processed);
		bool hasLineOnTheSides(const std_msgs::Int16MultiArrayConstPtr& processed, int height, int width);
		Action nextPathAction();

		void driveToAction(const pioneer_control::NavigationDriveToGoalConstPtr& goal);
		NavigationExecutePathServer navExecutePathServer;
		//NavigationExecutePathClient navExecutePathClient;
		pioneer_control::NavigationExecutePathFeedback navExecutePathFeedback;
		pioneer_control::NavigationExecutePathResult navExecutePathResult;

		void executePathAction(const pioneer_control::NavigationExecutePathGoalConstPtr& goal);
		NavigationDriveToServer navDriveToServer;
		pioneer_control::NavigationDriveToFeedback navDriveToFeedback;
		pioneer_control::NavigationDriveToResult navDriveToResult;
};

Navigation::Action Navigation::nextPathAction()
{
	Action front = directions.front();
	executedDirections.push_back(front);
	directions.pop_front();
	return front;
}

double linearInterpolation(double min, double t, double max)
{return (1 - t) * min + t * max;}

double quadraticInterpolation(double min, double t, double max)
{return (1 - t*ABS(t)) * min + t*ABS(t) * max;}

double cubicInterpolation(double min, double t, double max)
{return (1 - t*t*t) * min + t*t*t * max;}

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
		action = nextPathAction();
		isCrossing = true;
		switch (action) {
		case Action_turn_right: printf("turn_right\n"); break;
		case Action_turn_left: printf("turn_left\n"); break;
		case Action_go_straight: printf("go_straight\n");
		case Action_follow_line: break;
		}
	}

	switch (action) {
	case Action_stop:
		driveApi.setDrive(0, 0);
		break;
	case Action_turn_right:
		//printf("turn_right\n");
		driveApi.setDrive(0.055, 0.4);
		if(total_sum <= 28) {
			action = Action_follow_line;
			isCrossing = false;
		}
		break;
	case Action_turn_left:
		//printf("turn_left\n");
		driveApi.setDrive(0.055, -0.4);
		if(total_sum <= 28) {
			action = Action_follow_line;
			isCrossing = false;
		}
		break;
		directions.size();

	case Action_go_straight:
	case Action_follow_line:
		//printf("go_straight\n");

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
	}
}

void Navigation::testExecutePath()
{
	
}

void Navigation::driveToAction(const pioneer_control::NavigationDriveToGoalConstPtr& goal)
{
	//calcular path
	//transformar em directions
	//chamar execute path action
	//forward to feedback
}

void Navigation::waitForLocalizationChange()
{
	localizationLock = true;
	while(!localizationLock);
}

void Navigation::handleLocalizationChange(const geometry_msgs::Pose2D localization)
{
	if(!localizationLock)
	{
		local.x = localization.x;
		local.y = localization.y;
		localizationLock = false;
	}
}

bool Navigation::checkPath()
{
	return true;
}

void Navigation::executePathAction(const pioneer_control::NavigationExecutePathGoalConstPtr& goal)
{
	directions.clear();
	for(int i = 0; i < goal->path.size(); i++)
	{
		directions.push_back((Action)goal->path[i]);
	}

	navExecutePathFeedback.progress = 0.0;
	navExecutePathServer.publishFeedback(navExecutePathFeedback);
	int directionsTotal;
	isNavigationOn = true;
	while(!directions.empty())
	{
		//wait for localization 
		waitForLocalizationChange();
		if(!checkPath())
		{
			navExecutePathServer.setPreempted();
			//wrong way - cancel executePath and set not succeded
		}
		directionsTotal = directions.size() + executedDirections.size();
		navExecutePathFeedback.progress = executedDirections.size()/directionsTotal;
		navExecutePathServer.publishFeedback(navExecutePathFeedback);
	}
	navExecutePathResult.status = true;
	navExecutePathServer.setSucceeded(navExecutePathResult);

}

Navigation::Navigation(ros::NodeHandle n) : 
	navDriveToServer(node, "drive_to", boost::bind(&Navigation::driveToAction, this, _1), false),
	navExecutePathServer(node, "execute_path", boost::bind(&Navigation::executePathAction, this, _1), false),
	//navExecutePathClient("execute_path", false),
	driveApi(n)
{
	node = n;
	processedImageSub = node.subscribe<std_msgs::Int16MultiArray>(IMAGE_PROCESSED_TOPIC, 1,  &Navigation::handleProcessedImage, this);
	localizationSub = node.subscribe<geometry_msgs::Pose2D>(LOCALIZATION_TOPIC, 10, &Navigation::handleLocalizationChange, this);
	navDriveToServer.start();
	//percentage covered in black
	identifyCrossingThreshold = 0.6;
	stopTurningThreshold = 0.44;
	
	isCrossing = false;
	action = Action_follow_line;
	isNavigationOn = false;
	navExecutePathServer.start();
	ROS_INFO("Server started!");
	//testExecutePath();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation");
	ros::NodeHandle node;
	
	Navigation navigation(node);
	ros::spin();
	return 0;
}
