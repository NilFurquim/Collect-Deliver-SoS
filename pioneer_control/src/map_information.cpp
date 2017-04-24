#include "ros/ros.h"

//Services
#include "pioneer_control/MapInformationGetMap.h"
#include "pioneer_control/MapInformationGetProductAreas.h"
#include "pioneer_control/MapInformationUpdateMap.h"

//Msgs
#include "pioneer_control/PoseGrid.h"
#include "pioneer_control/UpdateMapMsg.h"
#include "pioneer_control/PAInformation.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

//Utils
#include "pioneer_control/Vec2.h"
#include <map>
#include <string>

#define MAP_INFORMATION_GET_MAP_SERV "/get_map"
#define MAP_INFORMATION_UPDATE_MAP_SERV "/update_map"
#define MAP_INFORMATION_POS_CHANGE_TOPIC "/pos_change"

class MapInformation
{
	public:
		MapInformation(ros::NodeHandle node);
	private:
		ros::NodeHandle node;

		pioneer_control::UpdateMapMsg posChangeMsg;
		ros::Publisher posChangePub;
		class RobotPosition {
			public:
				RobotPosition(unsigned id, unsigned x, unsigned y)
					: id(id), pos(x, y) {};
				RobotPosition(unsigned id, unsigned x, unsigned y, 
						int dx, int dy) 
					: id(id), pos(x, y), dir(dx, dy) {};
				Vec2i pos;
				Vec2i dir;
				unsigned id;
		};
		void printMap();
		std::vector<RobotPosition> robotPositions;
		static int const width = 7;
		static int const height = 7;
		
		static const std::map<std::string, Vec2i> pAreaPos;
		static std::map<std::string, Vec2i> initMap();
		static int matrix[height][width];

		ros::ServiceServer updateMapService;
		bool updateMap(pioneer_control::MapInformationUpdateMap::Request& req,
				pioneer_control::MapInformationUpdateMap::Response& res);

		ros::ServiceServer getMapService;
		bool getMap(pioneer_control::MapInformationGetMap::Request& req,
				pioneer_control::MapInformationGetMap::Response& res);

		ros::ServiceServer getProductAreasService;
		bool getProductAreas(pioneer_control::MapInformationGetProductAreas::Request& req,
				pioneer_control::MapInformationGetProductAreas::Response& res);

		int isOcupied(int i, int j);
		int hasUpConnection(int i, int j);
		int hasRightConnection(int i, int j);
		int hasDownConnection(int i, int j);
		int hasLeftConnection(int i, int j);
};

int MapInformation::matrix[height][width] = {
	{0,  4,  4,  4,  4,  4,  0},
	{2, 15, 15, 15, 15, 15,  8}, 
	{2, 15, 15, 15, 15, 15,  8}, 
	{2, 15, 15, 15, 15, 15,  8}, 
	{2, 15, 15, 15, 15, 15,  8}, 
	{2, 15, 15, 15, 15, 15,  8}, 
	{0,  1,  1,  1,  1,  1,  0} 
};

const std::map<std::string, Vec2i>MapInformation::pAreaPos = MapInformation::initMap();

std::map<std::string, Vec2i> MapInformation::initMap()
{
	std::map<std::string, Vec2i> pAreaPoses;
	pAreaPoses["A0"] = Vec2i(1,0);
	pAreaPoses["A1"] = Vec2i(2,0);
	pAreaPoses["A2"] = Vec2i(3,0);
	pAreaPoses["A3"] = Vec2i(4,0);
	pAreaPoses["A4"] = Vec2i(5,0);

	pAreaPoses["B0"] = Vec2i(6,1);
	pAreaPoses["B1"] = Vec2i(6,2);
	pAreaPoses["B2"] = Vec2i(6,3);
	pAreaPoses["B3"] = Vec2i(6,4);
	pAreaPoses["B4"] = Vec2i(6,5);

	pAreaPoses["C0"] = Vec2i(1,6);
	pAreaPoses["C1"] = Vec2i(2,6);
	pAreaPoses["C2"] = Vec2i(3,6);
	pAreaPoses["C3"] = Vec2i(4,6);
	pAreaPoses["C4"] = Vec2i(5,6);

	pAreaPoses["D0"] = Vec2i(0,1);
	pAreaPoses["D1"] = Vec2i(0,2);
	pAreaPoses["D2"] = Vec2i(0,3);
	pAreaPoses["D3"] = Vec2i(0,4);
	pAreaPoses["D4"] = Vec2i(0,5);
	return pAreaPoses;
}
////1////
//8 O 2//
////4////
//..RULD

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
	if (req.info.pose.pos.x >= width || req.info.pose.pos.y >= height)
	{
		ROS_INFO("Invalid x, y value for updateMap. \
				Values have to be x < %u and y < %u, \
				but were x = %u and y = %u\n", width, height, req.info.pose.pos.x, req.info.pose.pos.y);
		res.status = -1;
		return false;
	}

#define _ABS(var) ((var>0)?var:-var)
	int robotFound = false;
	Vec2i prev;

	for (int i = 0; i < robotPositions.size(); i++)
	{
		if (robotPositions[i].id == req.info.id)
		{
			prev = robotPositions[i].pos;
			robotPositions[i].pos.x = req.info.pose.pos.x;
			robotPositions[i].pos.y = req.info.pose.pos.y;
			robotPositions[i].dir.x = req.info.pose.dir.x;
			robotPositions[i].dir.y = req.info.pose.dir.y;
			matrix[req.info.pose.pos.y][req.info.pose.pos.x] = -_ABS(matrix[req.info.pose.pos.y][req.info.pose.pos.x]);
			posChangeMsg.id = req.info.id;
			posChangeMsg.pose.dir.x = req.info.pose.dir.x;
			posChangeMsg.pose.dir.y = req.info.pose.dir.y;
			posChangeMsg.pose.pos.x = req.info.pose.pos.x;
			posChangeMsg.pose.pos.y = req.info.pose.pos.y;
			posChangePub.publish(posChangeMsg);
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
		printMap();
		return true;
	}
	else
	{
		robotPositions.push_back(RobotPosition(req.info.id, req.info.pose.pos.x, req.info.pose.pos.y));
		matrix[req.info.pose.pos.y][req.info.pose.pos.x] = -_ABS(matrix[req.info.pose.pos.y][req.info.pose.pos.x]);
		posChangeMsg.id = req.info.id;
		posChangeMsg.pose.dir.x = req.info.pose.dir.x;
		posChangeMsg.pose.dir.y = req.info.pose.dir.y;
		posChangeMsg.pose.pos.x = req.info.pose.pos.x;
		posChangeMsg.pose.pos.y = req.info.pose.pos.y;
		posChangePub.publish(posChangeMsg);
		res.status = 0;
		printMap();
		return true;
	}
}

void MapInformation::printMap()
{
	printf("\n");
	for(int i = 0; i < height; i++)
	{
		printf("| ");
		for(int j = 0; j < width; j++)
		{
			if(matrix[i][j] < 0) printf("@ ");
			else printf(". ");
		}
		printf("|\n");
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
	//ROS_INFO("Map passed successfully!");
	return true;
}

bool MapInformation::getProductAreas(pioneer_control::MapInformationGetProductAreas::Request& req,
				pioneer_control::MapInformationGetProductAreas::Response& res)
{
	pioneer_control::PAInformation pAInfo;
	for(std::map<std::string, Vec2i>::const_iterator it = pAreaPos.begin(); it != pAreaPos.end(); it++)
	{
		pAInfo.name = it->first;
		pAInfo.pos.x = it->second.x;
		pAInfo.pos.y = it->second.y;
		res.pAs.push_back(pAInfo);
	}

	return true;
}

MapInformation::MapInformation(ros::NodeHandle n)
{
	node = n;
	initMap();
	updateMapService = node.advertiseService(MAP_INFORMATION_UPDATE_MAP_SERV, &MapInformation::updateMap, this);
	getMapService = node.advertiseService(MAP_INFORMATION_GET_MAP_SERV, &MapInformation::getMap, this);
	getProductAreasService = node.advertiseService("/get_pas", &MapInformation::getProductAreas, this);

	posChangePub = node.advertise<pioneer_control::UpdateMapMsg>(MAP_INFORMATION_POS_CHANGE_TOPIC, 10); 
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_information");
	ros::NodeHandle node;
	
	MapInformation mapInformation(node);
	ros::spin();
	return 0;
}
