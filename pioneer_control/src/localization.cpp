#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "pioneer_control/image_processing.h"
#include "nav_msgs/Odometry.h"
#include "pioneer_control/MapInformationGetMap.h"
#include "geometry_msgs/Pose2D.h"
#include "pioneer_control/Vec2.h"
#include "pioneer_control/Vec2i32.h"
#include "pioneer_control/PoseGrid.h"
#include "pioneer_control/LocalizationInit.h"

#define LOCALIZATION_TOPIC "localization"

class Localization
{
	public:
		Localization(ros::NodeHandle n);
	private:
		ros::NodeHandle node;
		ros::Subscriber processedImageSub;
		ros::Subscriber odometrySub;

		ros::Publisher localizationPub;
		pioneer_control::PoseGrid localizationPubMsg;

		ros::ServiceServer localizationInitService;
		ros::ServiceClient getMapClient;
		pioneer_control::MapInformationGetMap getMapService;
		bool initService(pioneer_control::LocalizationInit::Request& req,
				pioneer_control::LocalizationInit::Response& res);

		Vec2i static const dirs[4];
		Vec2i currentPosition;

		void publishCurrentDirection();
		bool validatePosition(Vec2i pos);
		bool validateDirection(Vec2i dir);
		int currentDirection;
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
	for(int i = 0; i < 5; i++)
	{
		printf("| ");
		for(int j = 0; j < 5; j++)
		{
			///if(mapMatrix[i][j] < 0)
			//	printf("  ");
			/*else*/ if(y == i && x == j)
				printf("@ ");
			else printf(". ");
		}
		printf("|\n");
	}
}

Vec2i const Localization::dirs[] = {
	Vec2i( 1, 0), 
	Vec2i( 0,-1), 
	Vec2i(-1, 0), 
	Vec2i( 0, 1)
};

Localization::Localization(ros::NodeHandle n)
{
	node = n;
	processedImageSub = node.subscribe<std_msgs::Int16MultiArray>(IMAGE_PROCESSED_TOPIC, 1,  &Localization::handleProcessedImage, this);
	odometrySub = node.subscribe<nav_msgs::Odometry>("diff_drive/odom", 1,  &Localization::handleOdometry, this);
	getMapClient = node.serviceClient<pioneer_control::MapInformationGetMap>("/get_map");

	localizationInitService = node.advertiseService("localization/init", &Localization::initService, this);
	localizationPub = node.advertise<pioneer_control::PoseGrid>(LOCALIZATION_TOPIC, 1); 
	currentPosition = Vec2i(1, 1);
	currentDirection = 3;
	mapHeight = -1;
	mapWidth = -1;

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

void Localization::publishCurrentDirection()
{
	localizationPubMsg.pos.x = currentPosition.x;
	localizationPubMsg.pos.y = currentPosition.y;
	localizationPubMsg.dir.x = dirs[currentDirection].x;
	localizationPubMsg.dir.y = dirs[currentDirection].y;
	printf("pos: (%d, %d) dir[%d](%d, %d)\n", currentPosition.x, currentPosition.y, currentDirection, dirs[currentDirection].x, dirs[currentDirection].y);
	//print_map(pos.x, pos.y);
	localizationPub.publish(localizationPubMsg);
}
bool Localization::validatePosition(Vec2i pos)
{
	if(pos.x < 0 || pos.y < 0)
		return false;
	if(pos.x >= mapWidth || pos.y >= mapHeight)
		return false;
	return true;
}

bool Localization::validateDirection(Vec2i dir)
{
	for(int i = 0; i < 4; i++)
		if(dir == dirs[i]) return true;
	return false;
}

bool Localization::initService(pioneer_control::LocalizationInit::Request& req,
				pioneer_control::LocalizationInit::Response& res)
{
	res.status = true;
	getMap();

	if(!validatePosition(Vec2i(req.pos.pos.x, req.pos.pos.y)))
	{
		ROS_INFO("Position (%d, %d) received is invalid! Position must be in the map (< (%d, %d))", req.pos.pos.x, req.pos.pos.y, mapWidth, mapHeight);
		res.status = false;
	}

	if(!validateDirection(Vec2i(req.pos.dir.x, req.pos.dir.y)))
	{
		ROS_INFO("Direction (%d, %d) is invalid! Direction must be (0,1), (0,-1), (1,0) or (-1,0)", req.pos.dir.x, req.pos.dir.y);
		res.status = false;
	}

	if(res.status)
	{
		ROS_INFO("Position and direction validated!");
		return true;
	}
	return false;

}

void Localization::getMap()
{
	if (getMapClient.call(getMapService))
	{
		std_msgs::Int32MultiArray respMap = getMapService.response.map;

		mapHeight = respMap.layout.dim[0].size;
		mapWidth = respMap.layout.dim[1].size;

		map = (int **)malloc (mapHeight * sizeof(int *));
		for (int i = 0; i < mapHeight; i++) {
			map[i] = (int *)malloc(mapWidth * sizeof(int));
		}


		for (int i = 0; i < mapHeight; i++) {
			for (int j = 0; j < mapWidth; j++) {
				map[i][j] = respMap.data[i*mapHeight + j];
			}
		}
		ROS_INFO("Map loaded successfully");
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
	linearTotal += twist.linear.x*dt;
	angularTotal += twist.angular.z*dt;

	if (linearTotal >= 0.3)
	{
		printf("angularTotal = %lf\n", angularTotal);
		if(angularTotal > 3.1415/2.1 || angularTotal < -3.1415/2.1)
		{
			printf("Localization: turned around\n");
			currentDirection =  (4 + currentDirection+2)%4;
		}
		else if(angularTotal > 3.1415/4)
		{
			printf("Localization: turned left\n");
			currentDirection =  (4 + currentDirection+1)%4;

		}
		else if(angularTotal < -3.1415/4)
		{
			currentDirection = (4 + currentDirection-1)%4;
			printf("Localization: turned right\n");
		}
		currentPosition += dirs[currentDirection];
		publishCurrentDirection();
		isCrossing = false;
		//angularTotal = 0;
		//linearTotal = 0;
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
		//printf("Is crossing\n");
		isCrossing = true;
		//Updating position
		//pos += dirs[currentDirection];
		//localizationPubMsg.x = pos.x;
		//localizationPubMsg.y = pos.y;
		//ROS_INFO("(%d, %d)", pos.x, pos.y);
		//print_map(pos.x, pos.y);
		//localizationPub.publish(localizationPubMsg);
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
