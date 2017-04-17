#include "ros/ros.h"

//Services
#include "pioneer_control/PathPlanningCalculateDistance.h"
#include "pioneer_control/GetLocalization.h"

//Msgs
#include "pioneer_control/Vec2i32.h"
#include "pioneer_control/TransportRequest.h"
#include "pioneer_control/ChooseClosestMsg.h"

//Utils
#include <string>

//Actions
#include "actionlib/client/terminal_state.h"
#include "actionlib/client/simple_action_client.h"
#include "pioneer_control/ControlGoPickUpProductAction.h"
typedef actionlib::SimpleActionClient<pioneer_control::ControlGoPickUpProductAction> ControlGoPickUpProduct;
#include "pioneer_control/ControlGoDeliverProductAction.h"
typedef actionlib::SimpleActionClient<pioneer_control::ControlGoDeliverProductAction> ControlGoDeliverProduct;

class RoboticAgent
{
	public:
		RoboticAgent(ros::NodeHandle n, int id);
	private:
		ros::NodeHandle node;

		int id;
		bool isTransporting, isClosest;
		int transportId;
		ros::Rate rate;

		ros::ServiceClient calculateDistanceClient;
		pioneer_control::PathPlanningCalculateDistance calculateDistanceService;

		ros::ServiceClient getLocalizationClient;
		pioneer_control::GetLocalization getLocalizationService;

		ros::Subscriber transportRequestSub;
		void handleTransportRequest(const pioneer_control::TransportRequestConstPtr& request);
		//if is closest call goPickUp action
		//	then call goDeliver action
		ros::Publisher chooseClosestPub;
		ros::Subscriber chooseClosestSub;
		pioneer_control::ChooseClosestMsg chooseClosestMsg;
		void handleChooseClosest(const pioneer_control::ChooseClosestMsgConstPtr& msg);

		pioneer_control::ControlGoPickUpProductGoal goPickUpProductGoal;
		ControlGoPickUpProduct goPickUpProductClient;

		pioneer_control::ControlGoDeliverProductGoal goDeliverProductGoal;
		ControlGoDeliverProduct goDeliverProductClient;
};

RoboticAgent::RoboticAgent(ros::NodeHandle n, int id)
	: goPickUpProductClient("pick_up_product", true),
	goDeliverProductClient("deliver_product", true),
	//TODO: ros::Rate durantion miliseconds?
	rate(3000)
{
	node = n;
	this->id = id;
	calculateDistanceClient = node.serviceClient<pioneer_control::PathPlanningCalculateDistance>("calculate_distance");
	getLocalizationClient = node.serviceClient<pioneer_control::GetLocalization>("get_localization");
	transportRequestSub = node.subscribe<pioneer_control::TransportRequest>("/transport_request", 0,  &RoboticAgent::handleTransportRequest, this);
	chooseClosestPub = node.advertise<pioneer_control::ChooseClosestMsg>("/choose_closest", 10);
	chooseClosestSub = node.subscribe<pioneer_control::ChooseClosestMsg>("/choose_closest", 10, &RoboticAgent::handleChooseClosest, this);

	isClosest = false;
	isTransporting = false;
}

void RoboticAgent::handleTransportRequest(const pioneer_control::TransportRequestConstPtr& request)
{
	//Am i the closest robot? 3 seconds to determine
	//isTransporting here should never happen
	if(!isTransporting)
	{
		ROS_INFO("Robot %d: In transport, transport %d denied.", id, request->id);
		return;
	}

	if(!getLocalizationClient.call(getLocalizationService))
	{
		ROS_INFO("Robot %d: Something's wrong with localization, transport %d denied.", id, request->id);
		return;
	}
	calculateDistanceService.request.origin = getLocalizationService.response.pose.pos;
	calculateDistanceService.request.goal = request->pickUpPA;
	if(!calculateDistanceClient.call(calculateDistanceService))
	{
		ROS_INFO("Robot %d: Something's wrong with path planning, transport %d denied.", id, request->id);
		return;
	}

	transportId = request->id;

	chooseClosestMsg.id = id;
	chooseClosestMsg.distance = calculateDistanceService.response.distance;
	chooseClosestPub.publish(chooseClosestMsg);
	//wait 3 seconds to decide wich robot is the closest
	rate.sleep();
	if(!isClosest)
	{
		ROS_INFO("Robot %d: Not the closest to pickUp PA, transport %d denied", id, request->id);
		return;
	}	

	isTransporting = true;
	goPickUpProductGoal.pAPos = request->pickUpPA;
	goPickUpProductClient.sendGoal(goPickUpProductGoal);
	goPickUpProductClient.waitForResult();
	//if result success

	goDeliverProductGoal.pAPos = request->pickUpPA;
	goDeliverProductClient.sendGoal(goDeliverProductGoal);
	goDeliverProductClient.waitForResult();
}

void RoboticAgent::handleChooseClosest(const pioneer_control::ChooseClosestMsgConstPtr& msg)
{
	if(msg->distance < chooseClosestMsg.distance)
	{
		isClosest = false;
		return;
	}

	if(msg->distance == chooseClosestMsg.distance && msg->id < id)
	{
		isClosest = false;
		return;
	}

	isClosest = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle node;
	int id = 0;
	RoboticAgent roboticAgent(node, id);
	
	ros::spin();
	return 0;
}
