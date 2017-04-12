#include "ros/ros.h"
#include "pioneer_control/MapInformationGetMap.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "pioneer_control/PathPlanningDefinePath.h"
#include "pioneer_control/PathPlanningCalculateDistance.h"
#include "pioneer_control/Vec2.h"
#include <queue>
#include <utility>
#include "pioneer_control/AStar.h"

#define PATH_PLANNING_DEFINE_PATH_SERVICE "define_path"
#define PATH_PLANNING_CALCULATE_DISTANCE_SERVICE "calculate_distance" 
class PathPlanning
{
	public:
		PathPlanning(ros::NodeHandle n);
	private:
		ros::NodeHandle node;

		int** map;
		int mapHeight, mapWidth;
		bool hasMap;

		ros::ServiceClient getMapClient;
		pioneer_control::MapInformationGetMap getMapService;
		void getMap();

		int distance(Vec2i origin, Vec2i goal);

		ros::ServiceServer calculateDistanceService;
		bool calculateDistance(pioneer_control::PathPlanningCalculateDistance::Request& req, 
				pioneer_control::PathPlanningCalculateDistance::Response& res);

		ros::ServiceServer definePathService;
		bool definePath(pioneer_control::PathPlanningDefinePath::Request& req, 
				pioneer_control::PathPlanningDefinePath::Response& res);
};

PathPlanning::PathPlanning(ros::NodeHandle n)
{
	node = n;
	definePathService = node.advertiseService(PATH_PLANNING_DEFINE_PATH_SERVICE, &PathPlanning::definePath, this);
	calculateDistanceService = node.advertiseService(PATH_PLANNING_CALCULATE_DISTANCE_SERVICE, &PathPlanning::calculateDistance, this);
	getMapClient = node.serviceClient<pioneer_control::MapInformationGetMap>("/get_map");
	getMap();
}

void PathPlanning::getMap()
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

		hasMap = true;
	}
	else
	{
		ROS_INFO("Couldn't load map from map information");
		hasMap = false;
	}

}

bool PathPlanning::definePath(pioneer_control::PathPlanningDefinePath::Request& req, pioneer_control::PathPlanningDefinePath::Response& res)
{
	getMap();
	if (!hasMap)
	{
		ROS_INFO("Couldn't load map from map information");
		return false;
	}
	Vec2i origin(req.origin.x, req.origin.y);
	Vec2i goal(req.goal.x, req.goal.y);
	//std::vector<Vec2i> path = Dijkstra(origin, goal);
	AStar aStar(map, mapHeight, mapWidth);
	std::vector<Vec2i> path = aStar.findPath(origin, goal);
	pioneer_control::Vec2i32 vec2i32_msg;
	for(std::vector<Vec2i>::iterator it = path.begin(); it < path.end(); it++)
	{
		vec2i32_msg.x = (*it).x;
		vec2i32_msg.y = (*it).y;
		res.path.push_back(vec2i32_msg);
		//printf("%d, %d\n", (*it).x, (*it).y);
	}
	
	return true;
}

#define ABS(var) ((var>0)?var:-var)
int PathPlanning::distance(Vec2i origin, Vec2i goal)
{
	Vec2i result = origin - goal;
	return ABS(result.x) + ABS(result.y);
}

bool PathPlanning::calculateDistance(pioneer_control::PathPlanningCalculateDistance::Request& req, pioneer_control::PathPlanningCalculateDistance::Response& res)
{
	res.distance = distance(Vec2i(req.origin.x,req.origin.y), Vec2i(req.goal.x, req.goal.y));
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle node;
	
	PathPlanning pathPlanning(node);
	ros::spin();
	return 0;
}
