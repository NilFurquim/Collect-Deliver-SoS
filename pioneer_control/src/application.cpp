#include "ros/ros.h"

#include "pioneer_control/ProductManagerAddRequest.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "application");
	ros::NodeHandle node;
	ros::ServiceClient addRequestClient = node.serviceClient<pioneer_control::ProductManagerAddRequest>("/add_request");
	pioneer_control::ProductManagerAddRequest addRequestService;
	char a;
	char pickUpPA[3];
	char deliverPA[3];
	pickUpPA[2] = '\0';
	deliverPA[2] = '\0';
	printf("To add a request use use X0->Y0, being X0 the PA where the product must be picked up, and Y0 the PA where the product must be delivered or type anything else to exit.\n");
	printf("Avaible PA's are: A0-A4, B0-B4, C0-C4 and D0-D4.\n");
	while(ros::ok())
	{
		if(scanf("%2s->%2s", pickUpPA, deliverPA) != 2)
		{
			printf("Are you sure you want to exit? Type \"y\" for yes, whatever else for no.\n");
			scanf("%*[^\n]");
			scanf("\n%c", &a);
			if(a == 'y')
			{
				printf("Exiting...\n");
				return 0;
			}
			else
			{
				scanf("%*[^\n]");
				continue;
			}
		}
		pickUpPA[2] = '\0';
		deliverPA[2] = '\0';
		addRequestService.request.pickUp = pickUpPA;
		addRequestService.request.deliver = deliverPA;
		if(!addRequestClient.call(addRequestService) || addRequestService.response.status == false)
		{
			ROS_INFO("Couldn't add request.");
		}
		else
		{
			ROS_INFO("Request added.");
		}
	}
	return 0;
}
