#include "ros/ros.h"

//Services
#include "pioneer_control/MapInformationGetProductAreas.h"
#include "pioneer_control/ProductManagerAddRequest.h"

//Msgs
#include "pioneer_control/TransportRequest.h"
#include "pioneer_control/TransportAccept.h"
#include "pioneer_control/TransportDone.h"

//Utils
#include "pioneer_control/Vec2.h"
#include <string>


class ProductManager
{
	public:
		ProductManager(ros::NodeHandle n);
		void run();
	private:
		ros::NodeHandle node;

		bool isDone;

		int transportCount;

		pioneer_control::TransportRequest transportRequestMsg;
		ros::Publisher transportRequestPub;

		pioneer_control::TransportAccept transportAcceptMsg;
		ros::Subscriber transportAcceptSub;
		void handleTransportAccept(const pioneer_control::TransportAcceptConstPtr& accept);

		pioneer_control::TransportDone transportDoneMsg;
		ros::Subscriber transportDoneSub;
		void handleTransportDone(const pioneer_control::TransportDoneConstPtr& done);
		
		ros::ServiceServer addRequestService;
		bool addRequest(pioneer_control::ProductManagerAddRequest::Request &req, pioneer_control::ProductManagerAddRequest::Response &res);

		ros::ServiceClient getProductAreasClient;
		pioneer_control::MapInformationGetProductAreas getProductAreasService;

		bool transportAccepted(int id);
		bool transportDone(int id);

		enum PAStatus {PAStatus_PickUp, PAStatus_Deliver, PAStatus_Idle, PAStatus_Empty};
		struct PAInformation
		{
			std::string name;
			Vec2i pos;
			enum PAStatus status;
		};
		std::map<std::string, struct PAInformation> productAreasInformation;

		class TransportInformation
		{
			public:
				TransportInformation();
				TransportInformation(int id, struct PAInformation *pickUpPA, struct PAInformation *deliverPA)
					: id(id), deliverPA(deliverPA), pickUpPA(pickUpPA), requestTime(ros::Time::now()) {};
				int id, robotId;
				ros::Time requestTime;
				ros::Time acceptTime;
				ros::Time doneTime;
				struct PAInformation *pickUpPA, *deliverPA;

		};
		std::vector<TransportInformation*> transportsWaitingAccept;
		std::vector<TransportInformation*> transportsWaitingResult;
		std::vector<TransportInformation*> transportsDone;

}; 

ProductManager::ProductManager(ros::NodeHandle n)
{
	node = n;
	transportCount = 0;
	isDone = false;
	getProductAreasClient = node.serviceClient<pioneer_control::MapInformationGetProductAreas>("/get_pas");
	transportRequestPub = node.advertise<pioneer_control::TransportRequest>("/transport_request", 0);
	transportAcceptSub = node.subscribe<pioneer_control::TransportAccept>("/transport_accept", 30, &ProductManager::handleTransportAccept, this);
	transportDoneSub = node.subscribe<pioneer_control::TransportDone>("/transport_done", 30, &ProductManager::handleTransportDone, this);
	addRequestService = node.advertiseService("/add_request", &ProductManager::addRequest, this);
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
		productAreasInformation["D1"].status = PAStatus_Idle;
		productAreasInformation["D2"].status = PAStatus_Idle;
		productAreasInformation["D3"].status = PAStatus_Idle;
		productAreasInformation["D4"].status = PAStatus_Idle;
	}
}

bool ProductManager::addRequest(pioneer_control::ProductManagerAddRequest::Request &req, pioneer_control::ProductManagerAddRequest::Response &res)
{
	std::string pickUp = req.pickUp;
	std::string deliver = req.deliver;
	res.status = false;
	bool isTransportValid = true;

	if(!productAreasInformation.count(pickUp)) {
		ROS_INFO("ERROR! Product Area name \"%s\" is not valid.", pickUp.c_str());
		isTransportValid = false;
	}
	if(!productAreasInformation.count(deliver)) {
		ROS_INFO("ERROR! Product Area name \"%s\" is not valid.", deliver.c_str());
		isTransportValid = false;
	}
	if(!isTransportValid) return false;
	PAInformation* pickUpPAInfo = &productAreasInformation[pickUp];
	PAInformation* deliverPAInfo = &productAreasInformation[deliver];
	switch(pickUpPAInfo->status)
	{
		case PAStatus_PickUp: 
			ROS_INFO("Pick Up PA (%s): ERROR! Product in this PA already to be picked up.", pickUp.c_str());
			isTransportValid = false;
			break;
		case PAStatus_Deliver:
			ROS_INFO("Pick Up PA (%s): ERROR! PA to be occupied.", pickUp.c_str());
			isTransportValid = false;
			break;
		case PAStatus_Empty:
			ROS_INFO("Pick Up PA (%s): ERROR! PA empty, no product here.", pickUp.c_str());
			isTransportValid = false;
			break;
		case PAStatus_Idle:
			ROS_INFO("Pick Up PA (%s): OK! Product waiting to be transported!.", pickUp.c_str());
			break;
	}

	switch(deliverPAInfo->status)
	{
		case PAStatus_PickUp: 
			ROS_INFO("Deliver PA (%s): ERROR! Product in this PA already to be picked up.", deliver.c_str());
			isTransportValid = false;
			break;
		case PAStatus_Deliver:
			ROS_INFO("Deliver PA (%s): ERROR! PA to be occupied.", deliver.c_str());
			isTransportValid = false;
			break;
		case PAStatus_Empty:
			ROS_INFO("Deliver PA (%s): OK! PA empty, product can be delivered here!.", deliver.c_str());
			break;
		case PAStatus_Idle:
			ROS_INFO("Deliver PA (%s): ERROR! Product here, can't deliver here!.", deliver.c_str());
			isTransportValid = false;
			break;
	}

	if(!isTransportValid) return false;

	TransportInformation* transportInfo;
	transportInfo = new TransportInformation(transportCount, pickUpPAInfo, deliverPAInfo);
	transportsWaitingAccept.push_back(transportInfo);

	ROS_INFO("Product Manager: Transport request %d (%s to %s) added to request queue.", transportCount, pickUp.c_str(), deliver.c_str());
	//transportRequestMsg.id = transportInfo->id;
	//transportRequestMsg.pickUpPA.x = transportInfo->pickUpPA->pos.x;
	//transportRequestMsg.pickUpPA.y = transportInfo->pickUpPA->pos.y;
	//transportRequestMsg.deliverPA.x = transportInfo->deliverPA->pos.x;
	//transportRequestMsg.deliverPA.y = transportInfo->deliverPA->pos.y;
	//printf("Product Manager: Request %d pick up from (%d, %d) ", transportCount, transportRequestMsg.pickUpPA.x, transportRequestMsg.pickUpPA.y);
	//printf("deliver to (%d, %d) published\n", transportRequestMsg.deliverPA.x, transportRequestMsg.deliverPA.y);
	//transportRequestPub.publish(transportRequestMsg);

	transportCount++;
	res.status = true;
	return true;
}

void ProductManager::handleTransportAccept(const pioneer_control::TransportAcceptConstPtr& accept)
{
	int id = accept->transportId;
	for(std::vector<TransportInformation*>::iterator it = transportsWaitingAccept.begin();
		       	it != transportsWaitingAccept.end(); it++)
	{
		if((*it)->id == id)
		{
			(*it)->pickUpPA->status = PAStatus_PickUp;
			(*it)->deliverPA->status = PAStatus_Deliver;
			(*it)->acceptTime = ros::Time::now();
			transportsWaitingResult.push_back(*it);
			transportsWaitingAccept.erase(it);
			ROS_INFO("Product Manager: Transport %d accepted.", id);
			return;
		}
	}
	return;
}

void ProductManager::handleTransportDone(const pioneer_control::TransportDoneConstPtr& done)
{
	int id = done->transportId;
	for(std::vector<TransportInformation*>::iterator it = transportsWaitingResult.begin();
		       	it != transportsWaitingResult.end(); it++)
	{
		if((*it)->id == id)
		{
			(*it)->pickUpPA->status = PAStatus_Empty;
			(*it)->deliverPA->status = PAStatus_Idle;
			(*it)->doneTime = ros::Time::now();
			transportsDone.push_back(*it);
			//fprintf(stderr, "%s->%s: (%lf\n", (*it)->pickUpPA->name.c_str(), (*it)->deliverPA->name.c_str(), doneTimeSec);
			double doneTimeSec = ((*it)->doneTime - (*it)->requestTime).toSec();
			std::cerr << (*it)->pickUpPA->name <<"->"<< (*it)->deliverPA->name << ", ";
			std::cerr << (*it)->requestTime << ", "<< (*it)->acceptTime << ", " << (*it)->doneTime << ", " << doneTimeSec << std::endl;
			transportsWaitingResult.erase(it);
			ROS_INFO("Product Manager: Transport %d done.", id);
			return;
		}
	}

	return;
}

void ProductManager::run()
{
	ros::Duration sleepDuration = ros::Duration(20);
	ros::Time init;
	ros::Duration interval;
	while(ros::ok() && !isDone)
	{
		//init = ros::Time::now();
		if(transportsWaitingAccept.size() > 0)
		{
			TransportInformation* first = transportsWaitingAccept.front();
			if(first->pickUpPA->status == PAStatus_Idle 
					&& first->deliverPA->status == PAStatus_Empty)
			{
				transportRequestMsg.id = first->id;
				transportRequestMsg.pickUpPA.x = first->pickUpPA->pos.x;
				transportRequestMsg.pickUpPA.y = first->pickUpPA->pos.y;
				transportRequestMsg.deliverPA.x = first->deliverPA->pos.x;
				transportRequestMsg.deliverPA.y = first->deliverPA->pos.y;
				printf("Product Manager: Request %d pick up from (%d, %d) ", transportRequestMsg.id, transportRequestMsg.pickUpPA.x, transportRequestMsg.pickUpPA.y);
				printf("deliver to (%d, %d) published\n", transportRequestMsg.deliverPA.x, transportRequestMsg.deliverPA.y);
				transportRequestPub.publish(transportRequestMsg);
			}
			else
			{
				printf("Couldn't publish request.\n");
			}
		}
		else
		{
			printf("Product Manager: Request queue empty.\n");
		}
		ros::Duration(3).sleep();
		//for(std::vector<TransportInformation*>::iterator it = transportsWaitingAccept.begin();
		//		it != transportsWaitingAccept.end(); it++)
		//{
		//	if((*it)->pickUpPA->status == PAStatus_Idle 
		//			&& (*it)->deliverPA->status == PAStatus_Empty)
		//	{
		//		transportRequestMsg.id = (*it)->id;
		//		transportRequestMsg.pickUpPA.x = (*it)->pickUpPA->pos.x;
		//		transportRequestMsg.pickUpPA.y = (*it)->pickUpPA->pos.y;
		//		transportRequestMsg.deliverPA.x = (*it)->deliverPA->pos.x;
		//		transportRequestMsg.deliverPA.y = (*it)->deliverPA->pos.y;
		//		printf("Product Manager: Request %d pick up from (%d, %d) ", transportRequestMsg.id, transportRequestMsg.pickUpPA.x, transportRequestMsg.pickUpPA.y);
		//		printf("deliver to (%d, %d) published\n", transportRequestMsg.deliverPA.x, transportRequestMsg.deliverPA.y);
		//		transportRequestPub.publish(transportRequestMsg);
		//		ros::Duration(0.01).sleep();
		//	}
		//}
		//interval = init - ros::Time::now();
		//if(interval < sleepDuration)
		//{
		//	(sleepDuration - interval).sleep();

		//}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "product_manager");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ProductManager productManager(node);
	productManager.run();
	return 0;
}
