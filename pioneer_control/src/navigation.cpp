#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "pioneer_control/image_processing.h"
#include "pioneer_control/NavigationDriveToAction.h"
#include "actionlib/server/simple_action_server.h"
#include "pioneer_control/DriveActuatorAPI.h"


#define ABS(var) ((var<0)? -var : var)
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

#define NAVIGATION_ON 1

class Navigation
{
	public:
		Navigation(ros::NodeHandle n);
	private:
		enum Action {Action_stop, Action_turn_right, Action_turn_left, Action_go_straight, Action_follow_line};
		ros::NodeHandle node;
		DriveActuatorAPI driveApi;
		ros::Subscriber processed_image_sub;
		bool is_navigation_on;
		Action action;
		double identify_crossing_threshold;
		
		void handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processed);
		Action nextPathAction();

		void driveToAction(const pioneer_control::NavigationDriveToGoalConstPtr& goal);
		NavigationDriveToServer navDriveToServer;
		pioneer_control::NavigationDriveToFeedback navDriveToFeedback;
		pioneer_control::NavigationDriveToResult nacDriveToResult;
};

Navigation::Action Navigation::nextPathAction()
{return Action_turn_left;}

double linear_interpolation(double min, double t, double max)
{return (1 - t) * min + t * max;}

double quadratic_interpolation(double min, double t, double max)
{return (1 - t*t) * min + t*t * max;}

double cubic_interpolation(double min, double t, double max)
{return (1 - t*t*t) * min + t*t*t * max;}

void Navigation::handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processedImg)
{
	if(!is_navigation_on) return;
	
	int width = processedImg->layout.dim[1].size;
	int height = processedImg->layout.dim[0].size;
	int offset = (height-1)*width;
	const short int *first_line = &processedImg->data[0];
	const short int *last_line = &processedImg->data[offset];

	int first_line_sum, last_line_sum, total_sum;
	float angular_regulator, linear_regulator;
	float first_line_avg, last_line_avg;
	float linear_speed = 0;

	bool is_crossing = false;

	total_sum = 0;
	for(int i = 0; i < height * width; i++)
		total_sum += processedImg->data[i];

	//for(int i = 0; i < height; i++)
	//{
	//	printf("| ");
	//	for(int j = 0; j < width; j++)
	//	{
	//		printf("%d ", processedImg->data[i*height + j]);
	//	}
	//	printf("|\n");
	//}
	//printf("\n");

	last_line_sum = 0;
	first_line_sum = 0;
	for(int i = 0; i < width; i++)
	{
		last_line_sum += last_line[i];
		first_line_sum += first_line[i];
	}
	//printf("width = %d\n", width);
	//printf("height = %d\n", height);
	//printf("total_sum = %d\n", total_sum);
	//stop if no guide line is identified
	if(total_sum == 0){
		//printf("sum == 0\n");
		driveApi.setDrive(0, 0);
		return;
	}

	//crossing reached
	//identify_crossing_threshold in percentage
	
	if(total_sum >= 38)
	{
		printf("Crossing reached\n");
		action = nextPathAction();
		is_crossing = true;
		linear_speed = 0.01;
	}


	int is_turn_complete = 1;
	switch (action) {
	case Action_stop:
		driveApi.setDrive(0, 0);
		break;
	case Action_turn_right:
		printf("turn_right\n");
		driveApi.setDrive(0.01, 1);
		if(total_sum <= 38) action = Action_follow_line;

#if 0
		is_turn_complete = 1;
		for(int i = offset; i < offset+width/2; i++)
		{
			if(processedImg->data[i] != 0){
				is_turn_complete = 0;
				break;
			}
		}
		if(is_turn_complete) action = Action_follow_line;
#endif
		break;
	case Action_turn_left:
		printf("turn_left\n");
		driveApi.setDrive(0.01, -1);
		if(total_sum <= 38) action = Action_follow_line;
#if 0
		is_turn_complete = 1;
		for(int i = offset+width/2+1; i < offset+width; i++)
		{
			if(processedImg->data[i] != 0){
				is_turn_complete = 0;
				break;
			}
		}
		if(is_turn_complete) action = Action_follow_line;
#endif
		break;
	case Action_go_straight:
	case Action_follow_line:

		for(int i = 0; i < width; i++)
		{
			first_line_avg += first_line[i]*(i+1-(width+1)/2.0);
			last_line_avg += last_line[i]*(i+1-(width+1)/2.0);
		}
		first_line_avg /= first_line_sum;
		last_line_avg /= last_line_sum;
		first_line_avg /= width/2.0;
		last_line_avg /= width/2.0;

		printf("first_line_avg = %f\n", first_line_avg);
		printf("last_line_avg = %f\n", last_line_avg);

		//normalized and centered
		if(first_line_sum == 0){
			angular_regulator = last_line_avg;
		}else{
			angular_regulator = first_line_avg * ABS(last_line_avg);
		}
		if(last_line_sum == 0) angular_regulator = 1;
		linear_regulator = 1 - ABS(angular_regulator);

		/*
		printf("avg=%f\n",avg);
		printf("total_sum=%d\n",total_sum);
		printf("norm=%f\n",normalized_avg);
		*/

		if(!is_crossing) linear_speed = cubic_interpolation(0.05, 
							linear_regulator, 0.35);
		driveApi.setDrive(linear_speed,
				  cubic_interpolation(0.05, angular_regulator, 1.7));
		break;
	}
}

void Navigation::driveToAction(const pioneer_control::NavigationDriveToGoalConstPtr& directions)
{
	//determine directions
	//GO
	//listen to localization while nextPath is not empty
	//	feedback
	//done
	bool action_is_done = false;

}

Navigation::Navigation(ros::NodeHandle n) : 
	navDriveToServer(node, "drive_to", boost::bind(&Navigation::driveToAction, this, _1), false),
	driveApi(n)
{
	node = n;
	processed_image_sub = node.subscribe<std_msgs::Int16MultiArray>(IMAGE_PROCESSED_TOPIC, 1,  &Navigation::handleProcessedImage, this);
	navDriveToServer.start();
	//percentage covered in black
	identify_crossing_threshold = 3/8 + .1;
	
	action = Action_follow_line;
	is_navigation_on = 1;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation");
	ros::NodeHandle node;
	
	Navigation navigation(node);
	ros::spin();
	return 0;
}
