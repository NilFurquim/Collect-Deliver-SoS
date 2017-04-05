#include "ros/ros.h"
#include "pioneer_control/MapInformationGetMap.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "pioneer_control/PathPlanningDefinePath.h"
#include "pioneer_control/PathPlanningCalculateDistance.h"
#include "pioneer_control/Vec2.h"

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

		std::vector<Vec2i> Dijkstra(Vec2i origin, Vec2i goal);
		std::vector<Vec2i> aStar(Vec2i origin, Vec2i goal);

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
	//std::vector<Vec2i> path = aStar(origin, goal);
	
	return true;
}

#define ABS(var) ((var>0)?var:-var)
bool PathPlanning::calculateDistance(pioneer_control::PathPlanningCalculateDistance::Request& req, pioneer_control::PathPlanningCalculateDistance::Response& res)
{
	if (map[req.origin.y][req.origin.x] == 0 && map[req.goal.y][req.goal.x] == 0)
	{
		res.distance = ABS(req.origin.x - req.goal.x) + ABS(req.origin.y - req.goal.y);
		res.distance /= 2;
		if(req.origin.x == req.goal.x) {
			if(req.origin.x == 0 || req.origin.x == mapWidth-1)
				res.distance += 2;
		} else if(req.origin.y == req.goal.y) {
			if(req.origin.y == 0 || req.origin.y == mapHeight-1)
				res.distance += 2;
		}
return true;
	}
	else
	{
		ROS_INFO("Not in a corssing!");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle node;
	
	PathPlanning pathPlanning(node);
	ros::spin();
	return 0;
}
