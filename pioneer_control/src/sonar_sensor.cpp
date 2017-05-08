#include "ros/ros.h"

//Msgs
#include "sensor_msgs/Range.h"
#include "pioneer_control/RangeArray.h"

//Services
//#include "pioneer_control/SonarSensorGetInformation.h"

#define FRONT_SENSORS_COUNT 4
class SonarSensor
{
	public:
		SonarSensor(ros::NodeHandle n);
		void start(double freq);
	private:
		ros::NodeHandle node;

		static const std::string frontSonarsTopics[FRONT_SENSORS_COUNT];
		ros::Subscriber frontSonarsSub[FRONT_SENSORS_COUNT];
		void handleSonarRange(const sensor_msgs::RangeConstPtr& range);

		pioneer_control::RangeArray frontSensorsMsg;
		ros::Publisher frontSensorsRangesPub;

		//ros::ServiceServer getInformationServer;
		//pioneer_control::SonarSensorGetInformation getInformationService;
		//bool getInformation(pioneer_control::SonarSensorGetInformation::Request &req,
		//		pioneer_control::SonarSensorGetInformation::Response &res);
};

const std::string SonarSensor::frontSonarsTopics[FRONT_SENSORS_COUNT] = {
	"sonar1",
	"sonar2",
	"sonar3",
	"sonar4"
};

SonarSensor::SonarSensor(ros::NodeHandle n)
	: node(n)
{
	frontSensorsRangesPub = node.advertise<pioneer_control::RangeArray>("front_sensors_ranges", 1);
	for(int i = 0; i < FRONT_SENSORS_COUNT; i++)
	{
		frontSonarsSub[i] = node.subscribe<sensor_msgs::Range>(frontSonarsTopics[i], 1, &SonarSensor::handleSonarRange, this);
	}

	for(int i = 0; i < FRONT_SENSORS_COUNT; i++)
	{
		frontSensorsMsg.ranges.push_back(-1.0);
	}
}

void SonarSensor::handleSonarRange(const sensor_msgs::RangeConstPtr& range)
{
	int i;
	std::string topicName = range->header.frame_id;
	for(i = 0; i < FRONT_SENSORS_COUNT; i++)
	{
		if(frontSonarsTopics[i].size() > topicName.size()) continue;

		std::string privateTopic(topicName.end() - frontSonarsTopics[i].size(), topicName.end());
		if(privateTopic == frontSonarsTopics[i]) break;
	}

	if(i == FRONT_SENSORS_COUNT)
	{ ROS_INFO("Sensor topic name error!"); }
	frontSensorsMsg.minRange = range->min_range;
	frontSensorsMsg.maxRange = range->max_range;
	frontSensorsMsg.ranges[i] = range->range;
}

void SonarSensor::start(double freq)
{
	ros::Rate rate(freq);
	pioneer_control::RangeArray sensors;
	sensor_msgs::RangeConstPtr sensorInfo;
	while(ros::ok())
	{
		frontSensorsRangesPub.publish(frontSensorsMsg);
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sonar_sensor");
	ros::NodeHandle node;
	SonarSensor sonarSensor(node);
	sonarSensor.start(10);
	return 0;
}
