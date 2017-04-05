#include "ros/ros.h"
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
		static int const width = 13;
		static int const height = 13;
		static int matrix[height][width];

		bool updateMap(pioneer_control::MapInformationUpdateMap::Request& req,
				pioneer_control::MapInformationUpdateMap::Response& res);
		bool getMap(pioneer_control::MapInformationGetMap::Request& req,
				pioneer_control::MapInformationGetMap::Response& res);
};

MapInformation::RobotPosition::RobotPosition(unsigned id, unsigned x, unsigned y)
	: id(id), pos(x, y) {}

// -1 = NO GUIDE LINE
// < -1 = GUIDE LINE OCUPIED -(weight+1)
// 0 = crossing
// > 0 weight of node

int MapInformation::matrix[height][width] = {
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
char const mapString[2*13*2*13+13+2*13+1] = {
"      .     .     .     .     .      \n\
      .     .     .     .     .      \n\
.  .  .  .  .  .  .  .  .  .  .  .  .\n\
      .     .     .     .     .      \n\
.  .  .  .  .  .  .  .  .  .  .  .  .\n\
      .     .     .     .     .      \n\
.  .  .  .  .  .  .  .  .  .  .  .  .\n\
      .     .     .     .     .      \n\
.  .  .  .  .  .  .  .  .  .  .  .  .\n\
      .     .     .     .     .      \n\
.  .  .  .  .  .  .  .  .  .  .  .  .\n\
      .     .     .     .     .      \n\
      .     .     .     .     .      \n"
};

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
	res.map.layout.dim.push_back(std_msgs::MultiArrayDimension());
	res.map.layout.data_offset = 0;
	res.map.layout.dim[0].label = "height";
	res.map.layout.dim[0].size = height;
	res.map.layout.dim[0].stride = 14;
	res.map.layout.dim.push_back(std_msgs::MultiArrayDimension());
	res.map.layout.dim[1].label = "width";
	res.map.layout.dim[1].size = width;
	res.map.layout.dim[1].stride = 1;
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			res.map.data.push_back(matrix[i][j]);
		}

	}
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
