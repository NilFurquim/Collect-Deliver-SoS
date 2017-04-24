#include "ros/ros.h"

//Services
#include "pioneer_control/MapInformationGetProductAreas.h"

//Msgs
#include "pioneer_control/TransportRequest.h"

//Utils
#include "pioneer_control/Vec2.h"
#include <string>


class Application
{
	public:
		Application(ros::NodeHandle n);
		bool requestTransport(std::string pickUp, std::string deliver);
	private:
		ros::NodeHandle node;

		int transportCount;

		pioneer_control::TransportRequest transportRequestMsg;
		ros::Publisher transportRequestPub;

		ros::ServiceClient getProductAreasClient;
		pioneer_control::MapInformationGetProductAreas getProductAreasService;

		enum PAStatus {PAStatus_PickUp, PAStatus_Deliver, PAStatus_Idle, PAStatus_Empty};
		struct PAInformation
		{
			std::string name;
			Vec2i pos;
			enum PAStatus status;
		};
		std::map<std::string, struct PAInformation> productAreasInformation;

		enum TransportStatus {TransportStatus_WaitingRobot, TransportStatus_PickUp, TransportStatus_Deliver};
		struct TransportInformation
		{
			int id, robotId;
			std::string from, to;
			enum TransportStatus status;
		};
		std::map<int, struct TransportInformation> transports;
}; 

Application::Application(ros::NodeHandle n)
{
	node = n;
	transportCount = 0;
	getProductAreasClient = node.serviceClient<pioneer_control::MapInformationGetProductAreas>("/get_pas");
	transportRequestPub = node.advertise<pioneer_control::TransportRequest>("/transport_request", 0);
	if(!getProductAreasClient.call(getProductAreasService))
	{
		ROS_INFO("Couldn't initialize product areas.");
	} else {
		typedef std::vector<pioneer_control::PAInformation> vectorPAInfo;
		vectorPAInfo pAInfos = getProductAreasService.response.pAs;
		struct PAInformation pAInfo;

		for(vectorPAInfo::iterator it = pAInfos.begin(); it != pAInfos.end(); it++)
		{
			pAInfo.name = it->name;
			pAInfo.pos = Vec2i(it->pos.x, it->pos.y);
			pAInfo.status = PAStatus_Empty;
			productAreasInformation[it->name] = pAInfo;
		}
		productAreasInformation["D0"].status = PAStatus_Idle;
	}
}

bool Application::requestTransport(std::string pickUp, std::string deliver)
{
	bool isTransportValid = true;
	if(!productAreasInformation.count(pickUp)) {
		ROS_INFO("Pick Up Product Area name not Valid.");
		isTransportValid = false;
	}
	if(!productAreasInformation.count(deliver)) {
		ROS_INFO("Deliver Product Area name not Valid.");
		isTransportValid = false;
	}
	if(!isTransportValid) return false;
	PAInformation pickUpPAInfo = productAreasInformation[pickUp];
	PAInformation deliverPAInfo = productAreasInformation[deliver];
	switch(pickUpPAInfo.status)
	{
		case PAStatus_PickUp: 
			ROS_INFO("Pick Up PA (%s): ERROR! Product in this PA already to be picked up.", pickUp.c_str());
			isTransportValid = false;
			break;
		case PAStatus_Deliver:
			ROS_INFO("Pick Up PA (%s): ERROR! PA to be occupied.", pickUp.c_str());
			isTransportValid = false;
isTransportValid = true;
			break;
		case PAStatus_Empty:
			ROS_INFO("Pick Up PA (%s): ERROR! PA empty, no product here.", pickUp.c_str());
			isTransportValid = false;
isTransportValid = true;
			break;
		case PAStatus_Idle:
			ROS_INFO("Pick Up PA (%s): OK! Product waiting to be transported!.", pickUp.c_str());
			break;
	}

	switch(deliverPAInfo.status)
	{
		case PAStatus_PickUp: 
			ROS_INFO("Deliver PA (%s): ERROR! Product in this PA already to be picked up.", deliver.c_str());
			isTransportValid = false;
			isTransportValid = true;
			break;
		case PAStatus_Deliver:
			ROS_INFO("Deliver PA (%s): ERROR! PA to be occupied.", deliver.c_str());
			isTransportValid = false;
isTransportValid = true;
			break;
		case PAStatus_Empty:
			ROS_INFO("Deliver PA (%s): OK! PA empty, product can be delivered here!.", deliver.c_str());
isTransportValid = true;
			break;
		case PAStatus_Idle:
			ROS_INFO("Deliver PA (%s): ERROR! Product here, can't deliver here!.", deliver.c_str());
			isTransportValid = false;
isTransportValid = true;
			break;
	}

	if(!isTransportValid) return false;

	pickUpPAInfo.status = PAStatus_PickUp;
	deliverPAInfo.status = PAStatus_Deliver;

	struct TransportInformation transportInfo;
	transportInfo.id = transportCount;
	transportInfo.from = pickUpPAInfo.name;
	transportInfo.to = deliverPAInfo.name;
	transportInfo.status = TransportStatus_WaitingRobot;

	transports[transportCount] = transportInfo;

	transportRequestMsg.id = transportCount;
	transportRequestMsg.pickUpPA.x = pickUpPAInfo.pos.x;
	transportRequestMsg.pickUpPA.y = pickUpPAInfo.pos.y;
	transportRequestMsg.deliverPA.x = deliverPAInfo.pos.x;
	transportRequestMsg.deliverPA.y = deliverPAInfo.pos.y;
	printf("Request %d: Pick up from (%d, %d) ", transportCount, transportRequestMsg.pickUpPA.x, transportRequestMsg.pickUpPA.y);
	printf("deliver to (%d, %d).\n", transportRequestMsg.deliverPA.x, transportRequestMsg.deliverPA.y);
	transportRequestPub.publish(transportRequestMsg);
	ROS_INFO("Transport request from %s to %s published!", 
						pickUp.c_str(), deliver.c_str());

	transportCount++;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle node;
	Application app(node);
	bool exit = false;
	std::string pickUpPA;
	std::string deliverPA;
	while(!exit)
	{
		std::getline(std::cin, pickUpPA);
		if(pickUpPA == "exit")
		{
			exit = true;	
			break;
		}
		std::getline(std::cin, deliverPA);
		if(pickUpPA == "exit")
		{
			exit = true;	
			break;
		}
		app.requestTransport(pickUpPA, deliverPA);
	}
	
	ros::spin();
	return 0;
}
