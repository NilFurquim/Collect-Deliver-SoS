#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "pioneer_control/map.h"
#include "pioneer_control/MapInformationUpdateMap.h"
#include "pioneer_control/MapInformationGetMap.h"
#include "pioneer_control/Vec2.h"


//updateMap()
//	update robot position
//getMap()
//	map
//subscribe()
//	tell positions updated

#define MAP_INFORMATION_GET_MAP_SERV "/get_map"
#define MAP_INFORMATION_UPDATE_MAP_SERV "/update_map"
#define MAP_INFORMATION_POS_CHANGE_TOPIC "/pos_change"

class MapInformation
{
	public:
		MapInformation(ros::NodeHandle node);
	private:
		ros::NodeHandle node;
		ros::ServiceServer updateMapService;
		ros::ServiceServer getMapService;
		ros::Publisher posChangePub;
		class RobotPosition {
			public:
				RobotPosition(unsigned id, unsigned x, unsigned y);
				Vec2i pos;
				unsigned id;
		};
		void printMap();
		std::vector<RobotPosition> robotPositions;
		static int const width = 7;
		static int const height = 7;
		
		//static int const width = 13;
		//static int const height = 13;
		static int matrix[height][width];

		bool updateMap(pioneer_control::MapInformationUpdateMap::Request& req,
				pioneer_control::MapInformationUpdateMap::Response& res);
		bool getMap(pioneer_control::MapInformationGetMap::Request& req,
				pioneer_control::MapInformationGetMap::Response& res);
		int isOcupied(int i, int j);
		int hasUpConnection(int i, int j);
		int hasRightConnection(int i, int j);
		int hasDownConnection(int i, int j);
		int hasLeftConnection(int i, int j);
};

MapInformation::RobotPosition::RobotPosition(unsigned id, unsigned x, unsigned y)
	: id(id), pos(x, y) {}

//int MapInformation::matrix[height][width] = {
//		{-1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1}, 
//		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
//		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
//		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
//		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
//		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
//		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
//		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
//		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
//		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
//		{ 0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,  3,  0}, 
//		{-1, -1,  3, -1,  3, -1,  3, -1,  3, -1,  3, -1, -1}, 
//		{-1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1}
//	};

int MapInformation::matrix[height][width] = {
	{0,  4,  4,  4,  4,  4,  0},
	{2, 15, 15, 15, 15, 15,  8}, 
	{2, 15, 15, 15, 15, 15,  8}, 
	{2, 15, 15, 15, 15, 15,  8}, 
	{2, 15, 15, 15, 15, 15,  8}, 
	{2, 15, 15, 15, 15, 15,  8}, 
	{0,  1,  1,  1,  1,  1,  0} 
};
////1////
//8 O 2//
////4////
//..RULD
//char const mapString[2*13*2*13+13+2*13+1] = {
//"      .     .     .     .     .      \n\
//      .     .     .     .     .      \n\
//.  .  .  .  .  .  .  .  .  .  .  .  .\n\
//      .     .     .     .     .      \n\
//.  .  .  .  .  .  .  .  .  .  .  .  .\n\
//      .     .     .     .     .      \n\
//.  .  .  .  .  .  .  .  .  .  .  .  .\n\
//      .     .     .     .     .      \n\
//.  .  .  .  .  .  .  .  .  .  .  .  .\n\
//      .     .     .     .     .      \n\
//.  .  .  .  .  .  .  .  .  .  .  .  .\n\
//      .     .     .     .     .      \n\
//      .     .     .     .     .      \n"
//};

int MapInformation::isOcupied(int i, int j)
{ return (matrix[i][j] < 0) ? 1 : 0; }

int MapInformation::hasUpConnection(int i, int j)
{ return (matrix[i][j] & 1) >> 0; }

int MapInformation::hasRightConnection(int i, int j)
{ return (matrix[i][j] & 2) >> 1; }

int MapInformation::hasDownConnection(int i, int j)
{ return (matrix[i][j] & 4) >> 2; }

int MapInformation::hasLeftConnection(int i, int j)
{ return (matrix[i][j] & 8) >> 3; }

bool MapInformation::updateMap(pioneer_control::MapInformationUpdateMap::Request& req,
		pioneer_control::MapInformationUpdateMap::Response& res)
{

	//warning: if two services are called ate the same time this is not thread safe!
	//warning2: this service is not optmized, for more robots optimizations would be convenient
	if (req.x >= width || req.y >= height)
	{
		ROS_INFO("Invalid x, y value for updateMap. \
				Values have to be x < %u and y < %u, \
				but were x = %u and y = %u\n", width, height, req.x, req.y);
		res.status = -1;
		return false;
	}

#define _ABS(var) ((var>0)?var:-var)
	int robotFound = false;
	Vec2i prev;

	for (int i = 0; i < robotPositions.size(); i++)
	{
		if (robotPositions[i].id == req.robotId)
		{
			prev = robotPositions[i].pos;
			robotPositions[i].pos.x = req.x;
			robotPositions[i].pos.y = req.y;
			matrix[req.y][req.x] = -_ABS(matrix[req.y][req.x]);
			robotFound = true;
		}
	}

	int robotsAtPos = 0;
	if (robotFound)
	{
		for (int i = 0; i < robotPositions.size(); i++)
		{
			if (robotPositions[i].pos == prev)
			{
				robotsAtPos++;
			}
		}
		if (robotsAtPos == 0) matrix[prev.y][prev.x] *= (-1);
		res.status = 0;
		return true;
	}
	else
	{
		robotPositions.push_back(RobotPosition(req.robotId, req.x, req.y));
		matrix[req.y][req.x] = -_ABS(matrix[req.y][req.x]);
		res.status = 0;
		return true;
	}



}

void MapInformation::printMap()
{
	for (int i = 0; i < robotPositions.size(); i++)
	{
	}
}

bool MapInformation::getMap(pioneer_control::MapInformationGetMap::Request& req,
		pioneer_control::MapInformationGetMap::Response& res)
{
	//TODO: Better map passing.
	//	UInt8MultiArray of 3 dimensions
	//	[0] = height
	//	[1] = width
	//	[2] = informations about node: connections and if is occupied
	res.map.layout.dim.push_back(std_msgs::MultiArrayDimension());
	res.map.layout.data_offset = 0;
	res.map.layout.dim[0].label = "height";
	res.map.layout.dim[0].size = height;
	res.map.layout.dim[0].stride = height;
	res.map.layout.dim.push_back(std_msgs::MultiArrayDimension());
	res.map.layout.dim[1].label = "width";
	res.map.layout.dim[1].size = width;
	res.map.layout.dim[1].stride = 1;
	//U: Up conection;
	//R: Right conection;
	//D: Down conection;
	//L: Left conection;
	//O: Ocupied;
	//res.map.layout.dim[2].label = "node config, U R D L O";
	//res.map.layout.dim[2].size = 5;
	//res.map.layout.dim[2].stride = 1;
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			//res.map.data.push_back(hasUpConnection(i, j));
			//res.map.data.push_back(hasRightConnection(i, j));
			//res.map.data.push_back(hasDownConnection(i, j));
			//res.map.data.push_back(hasLeftConnection(i, j));
			//res.map.data.push_back(isOcupied(i, j));
			res.map.data.push_back(matrix[i][j]);
		}

	}
	ROS_INFO("Map passed successfully!");
	return true;
}

MapInformation::MapInformation(ros::NodeHandle n)
{
	node = n;
	updateMapService = node.advertiseService(MAP_INFORMATION_UPDATE_MAP_SERV, &MapInformation::updateMap, this);
	getMapService = node.advertiseService(MAP_INFORMATION_GET_MAP_SERV, &MapInformation::getMap, this);
	//posChangePub = node.advertise(MAP_INFORMATION_POS_CHANGE_TOPIC); 
	
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_information");
	ros::NodeHandle node;
	
	MapInformation mapInformation(node);
	ros::spin();
	return 0;
}
