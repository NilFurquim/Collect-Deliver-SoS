#ifndef _IMAGE_PROCESSING_H_
#define _IMAGE_PROCEESING_H_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#define CHUNK_AMOUNT 8
#define IDENTIFY_BLACK_SIZE_THRESHOLD 8
#define IDENTIFY_BLACK_REDUCTION_FACTOR 1
#define IMAGE_PROCESSED_TOPIC "image_processing/processed"

class ImageProcessing
{
	public:
		ImageProcessing(ros::NodeHandle node);
		void handleImage(const sensor_msgs::ImageConstPtr& image);
		ros::Publisher handleImagePub;
	private:
		ros::NodeHandle node;
		int handleImage_recursive(
				const sensor_msgs::ImageConstPtr& image, 
				int x, int y, 
				int height, int width, 
				int chunkAmount, int sizeThreshold,
				float reductionFactor);
};
#endif
