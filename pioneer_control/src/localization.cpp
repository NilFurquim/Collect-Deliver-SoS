#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32.h"
#include "pioneer_control/image_processing.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <deque>
#include "pioneer_control/map.h"

#define LOCALIZATION_TOPIC "next_node"

void makeMap(Map& map);
class Localization
{
	public:
		Localization(ros::NodeHandle n);
	private:
		enum Direction{Direction_north = 0, Direction_east, Direction_south, Direction_west}; 
		ros::NodeHandle node;
		ros::Subscriber processedImageSub;
		ros::Subscriber odometrySub;
		ros::Publisher localizationPub;
		void handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processed);
		void handleOdometry(const nav_msgs::OdometryConstPtr& odom);
		bool isCrossing, isFollowing, isInit;
		double angularTotal, linearTotal;
		ros::Time prevTime;
		Direction actualDirection;
		std_msgs::Int32 localizationMsg;
};

Localization::Localization(ros::NodeHandle n)
{
	node = n;
	processedImageSub = node.subscribe<std_msgs::Int16MultiArray>(IMAGE_PROCESSED_TOPIC, 1,  &Localization::handleProcessedImage, this);
	odometrySub = node.subscribe<nav_msgs::Odometry>("diff_drive/odom", 1,  &Localization::handleOdometry, this);
	localizationPub = node.advertise<std_msgs::Int32>(LOCALIZATION_TOPIC, 1); 

	isCrossing = isFollowing = isInit = false;
	angularTotal = linearTotal = 0;

	
	prevTime = ros::Time::now();
}

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

void Localization::handleOdometry(const nav_msgs::OdometryConstPtr& odom)
{
	if(!isCrossing)
	{
		prevTime = ros::Time::now();
		return;
	}

	geometry_msgs::Twist twist = odom->twist.twist;
	double dt = (ros::Time::now() - prevTime).toSec();
	angularTotal += twist.angular.z*dt;
	linearTotal += twist.linear.x*dt;

	if(linearTotal >= 0.3)
	{
		printf("prev dir = %d\n", actualDirection);
		//printf("angularTotal = %lf\n", angularTotal);
		if(angularTotal > 3.1415/4)
		{
			//printf("Turned left\n");

		}
		else if(angularTotal < -3.1415/4)
		{
			//printf("Turned right\n");
		}
		isCrossing = false;
		printf("dir = %d\n", actualDirection);
	}
	prevTime = ros::Time::now();
	//printf("delta = %lf\n", (ros::Time::now() - prevTime).toSec());
}

void Localization::handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processedImg)
{
	int height = processedImg->layout.dim[0].size;
	int width = processedImg->layout.dim[1].size;
	int totalSum;

	totalSum = 0;
	for(int i = 0; i < height * width; i++)
		totalSum += processedImg->data[i];

	if(totalSum >= 0) isFollowing = true;
	else isFollowing = false;

	if(totalSum >= 40 && isCrossing==false)
	{
		printf("Is crossing\n");
		isCrossing = true;
		angularTotal = 0;
		linearTotal = 0;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "localization");
	ros::NodeHandle node;
	
	Localization localization(node);
	ros::spin();
	return 0;
}
