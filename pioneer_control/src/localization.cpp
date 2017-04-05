#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "pioneer_control/image_processing.h"
#include "nav_msgs/Odometry.h"
#include "pioneer_control/MapInformationGetMap.h"
#include "geometry_msgs/Pose2D.h"
#include "pioneer_control/Vec2.h"

#define LOCALIZATION_TOPIC "map_position"

class Localization
{
	public:
		Localization(ros::NodeHandle n);
	private:
		ros::NodeHandle node;
		ros::Subscriber processedImageSub;
		ros::Subscriber odometrySub;

		ros::Publisher localizationPub;
		geometry_msgs::Pose2D localizationPubMsg;

		ros::ServiceClient getMapClient;
		pioneer_control::MapInformationGetMap getMapService;

		Vec2i static const dirs[4];
		Vec2i pos;

		int actualdir;
		int** map;
		int mapWidth, mapHeight;
		void getMap();
		void handleProcessedImage(const std_msgs::Int16MultiArrayConstPtr& processed);
		void handleOdometry(const nav_msgs::OdometryConstPtr& odom);
		bool isCrossing, isFollowing, isInit;
		double angularTotal, linearTotal;
		ros::Time prevTime;
};

int mapMatrix[13][13] = {
		{-1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1}, 
		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
		{-1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1}
	};

void print_map(int x, int y)
{
	for(int i = 0; i < 13; i++)
	{
		printf("| ");
		for(int j = 0; j < 13; j++)
		{
			if(mapMatrix[i][j] < 0)
				printf("  ");
			else if(y == i && x == j)
				printf("@ ");
			else printf(". ");
		}
		printf("|\n");
	}
}

Vec2i const Localization::dirs[] = {
	Vec2i(1,0), 
	Vec2i(0,-1), 
	Vec2i(-1,0), 
	Vec2i(0, 1)
};

Localization::Localization(ros::NodeHandle n)
{
	node = n;
	processedImageSub = node.subscribe<std_msgs::Int16MultiArray>(IMAGE_PROCESSED_TOPIC, 1,  &Localization::handleProcessedImage, this);
	odometrySub = node.subscribe<nav_msgs::Odometry>("diff_drive/odom", 1,  &Localization::handleOdometry, this);
	getMapClient = node.serviceClient<pioneer_control::MapInformationGetMap>("/get_map");
	ROS_INFO("Before get map");
	//getMap();

	localizationPub = node.advertise<geometry_msgs::Pose2D>(LOCALIZATION_TOPIC, 1); 
	pos = Vec2i(2, 1);
	actualdir = 3;

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

void Localization::getMap()
{
	if (getMapClient.call(getMapService))
	{
		std_msgs::Int32MultiArray respMap = getMapService.response.map;

		mapHeight = respMap.layout.dim[0].size;
		mapWidth = respMap.layout.dim[1].size;

		map = (int **)malloc (mapHeight * sizeof(int *));
		for (int i = 0; i < mapWidth; i++) {
			map[i] = (int *)malloc(mapHeight * sizeof(int));
		}


		for (int i = 0; i < mapHeight; i++) {
			for (int j = 0; j < mapHeight; j++) {
				map[i][j] = respMap.data[i*mapWidth + j];
			}
		}
	}
	else
	{
		ROS_INFO("Could not call service getMap");
	}

}

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
		//printf("angularTotal = %lf\n", angularTotal);
		if(angularTotal > 3.1415/4)
		{
			//printf("Turned left\n");
			actualdir =  (4 + actualdir+1)%4;

		}
		else if(angularTotal < -3.1415/4)
		{
			actualdir = (4 + actualdir-1)%4;
			//printf("Turned right\n");
		}
		pos += dirs[actualdir];
		localizationPubMsg.x = pos.x;
		localizationPubMsg.y = pos.y;
		ROS_INFO("pos: (%d, %d) dir[%d](%d, %d)", pos.x, pos.y, actualdir, dirs[actualdir].x, dirs[actualdir].y);
		print_map(pos.x, pos.y);
		localizationPub.publish(localizationPubMsg);
		isCrossing = false;
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
		pos += dirs[actualdir];
		localizationPubMsg.x = pos.x;
		localizationPubMsg.y = pos.y;
		ROS_INFO("(%d, %d)", pos.x, pos.y);
		print_map(pos.x, pos.y);
		localizationPub.publish(localizationPubMsg);
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
