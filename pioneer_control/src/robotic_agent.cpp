#include "ros/ros.h"

//Services
#include "pioneer_control/PathPlanningCalculateDistance.h"
#include "pioneer_control/GetLocalization.h"

//Msgs
#include "pioneer_control/Vec2i32.h"
#include "pioneer_control/ChooseClosestMsg.h"
#include "pioneer_control/TransportRequest.h"
#include "pioneer_control/TransportAccept.h"
#include "pioneer_control/TransportDone.h"

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
		int distance;

		ros::ServiceClient calculateDistanceClient;
		pioneer_control::PathPlanningCalculateDistance calculateDistanceService;

		ros::ServiceClient getLocalizationClient;
		pioneer_control::GetLocalization getLocalizationService;

		void subscribeToTransportRequest();
		ros::Subscriber transportRequestSub;
		void handleTransportRequest(const pioneer_control::TransportRequestConstPtr& request);
		ros::Publisher transportDonePub;
		pioneer_control::TransportDone transportDoneMsg;
		ros::Publisher transportAcceptPub;
		pioneer_control::TransportAccept transportAcceptMsg;

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
	: goPickUpProductClient("go_pick_up_product", true),
	goDeliverProductClient("go_deliver_product", true)
{
	node = n;
	this->id = id;
	calculateDistanceClient = node.serviceClient<pioneer_control::PathPlanningCalculateDistance>("calculate_distance");
	getLocalizationClient = node.serviceClient<pioneer_control::GetLocalization>("get_localization");
	chooseClosestSub = node.subscribe<pioneer_control::ChooseClosestMsg>("/choose_closest", 30, &RoboticAgent::handleChooseClosest, this);
	chooseClosestPub = node.advertise<pioneer_control::ChooseClosestMsg>("/choose_closest", 30);

	subscribeToTransportRequest();
	transportDonePub = node.advertise<pioneer_control::TransportDone>("/transport_done", 5);
	transportAcceptPub = node.advertise<pioneer_control::TransportAccept>("/transport_accept", 5);
	
	isClosest = false;
	isTransporting = false;
	distance = 10000000;
}

void RoboticAgent::subscribeToTransportRequest()
{
	transportRequestSub = node.subscribe<pioneer_control::TransportRequest>("/transport_request", 100,  &RoboticAgent::handleTransportRequest, this);
}

void RoboticAgent::handleTransportRequest(const pioneer_control::TransportRequestConstPtr& request)
{
	transportRequestSub.shutdown();
	//Am i the closest robot? 2 seconds to determine
	//isTransporting here should never happen
	if(isTransporting)
	{
		ROS_INFO("Robot %d: In transport, transport %d denied.", id, request->id);
		subscribeToTransportRequest();
		return;
	}

	if(!getLocalizationClient.call(getLocalizationService))
	{
		ROS_INFO("Robot %d: Something's wrong with localization, transport %d denied.", id, request->id);
		subscribeToTransportRequest();
		return;
	}
	calculateDistanceService.request.origin = getLocalizationService.response.pose.pos;
	calculateDistanceService.request.goal = request->pickUpPA;
	if(!calculateDistanceClient.call(calculateDistanceService))
	{
		ROS_INFO("Robot %d: Something's wrong with path planning, transport %d denied.", id, request->id);
		subscribeToTransportRequest();
		return;
	}

	transportId = request->id;

	chooseClosestMsg.id = id;
	chooseClosestMsg.distance = calculateDistanceService.response.distance;
	distance = calculateDistanceService.response.distance;
	isClosest = true;
	chooseClosestPub.publish(chooseClosestMsg);

	for(int i = 0; i < 8; i++)
	{
		ros::spinOnce();
		ros::Duration(0.25).sleep();
	}
	//wait 3 seconds to decide wich robot is the closest
	if(!isClosest)
	{
		ROS_INFO("Robot %d: Not the closest to pickUp PA, transport %d denied", id, request->id);
		subscribeToTransportRequest();
		return;
	}	

	ROS_INFO("Robot %d: Transport %d accepted", id, request->id);
	transportAcceptMsg.transportId = request->id;
	transportAcceptPub.publish(transportAcceptMsg);

	isTransporting = true;
	goPickUpProductClient.waitForServer();
	goPickUpProductGoal.pAPos = request->pickUpPA;
	goPickUpProductClient.sendGoal(goPickUpProductGoal);
	goPickUpProductClient.waitForResult();
	//if result success

	goDeliverProductClient.waitForServer();
	goDeliverProductGoal.pAPos = request->deliverPA;
	goDeliverProductClient.sendGoal(goDeliverProductGoal);
	goDeliverProductClient.waitForResult();
	isTransporting = false;
	transportDoneMsg.transportId = transportId;
	ROS_INFO("Robot agent done with %d.", transportId);
	transportDonePub.publish(transportDoneMsg);
	subscribeToTransportRequest();
}

void RoboticAgent::handleChooseClosest(const pioneer_control::ChooseClosestMsgConstPtr& msg)
{
	//ROS_INFO("Handle Choose closest id %d, dist %d", msg->id, msg->distance);
	if(!isClosest || msg->id == id) return;

	if(msg->distance < distance)
	{
		printf("%d, not the closest.\n", id);
		isClosest = false;
		return;
	}

	if(msg->distance == chooseClosestMsg.distance && msg->id < id)
	{
		printf("%d, not the closest.\n", id);
		isClosest = false;
		return;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotic_agent");
	ros::NodeHandle node;
	if(argc != 2)
	{
		ROS_INFO("Wrong number of args, requires id");
		return 1;
	}
	RoboticAgent roboticAgent(node, atoi(argv[1]));
	
	ros::spin();
	return 0;
}
