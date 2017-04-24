#include "pioneer_control/image_processing.h"
#include "std_msgs/Int16MultiArray.h"

#define PRINT_MATRIX(matrix, size) for(int i = 0; i < size; i++){\
		printf("\n|");\
		for(int j = 0; j < size; j++){\
			printf((matrix[i][j] == 1)?"0":" ");\
		}printf("|");}printf("\n");

ImageProcessing::ImageProcessing(ros::NodeHandle n)
{
	node = n;
	handleImagePub = n.advertise<std_msgs::Int16MultiArray>(IMAGE_PROCESSED_TOPIC, 1);
}

int ImageProcessing::handleImage_recursive(
		const sensor_msgs::ImageConstPtr& image, 
		int x, int y, int height, int width, 
		int chuckAmount, int sizeThreshold,
		float reductionFactor)
{
	if(height <= sizeThreshold || width <= sizeThreshold)
	{
		int mid = (y + height/2) * image->step + (x + width/2)*3;
		uint8_t mid_red = image->data[mid];
		uint8_t mid_green = image->data[mid + 1];
		uint8_t mid_blue = image->data[mid + 2];

		if(mid_red < 6 && mid_green < 6 && mid_blue < 6)
			return 1;
		return 0;
	}

	int deslx, desly;
	int chunkHeight = height/chuckAmount;
	int chunkWidth = width/chuckAmount;
	int isBlack = 0;
	int count = 0;

	int reduction_size = chuckAmount/reductionFactor;
	for(int i = 0; i < reduction_size; i++)
	{
		for(int j = 0; j < reduction_size; j++)
		{
			desly = chunkHeight*(i + (chuckAmount - reduction_size)/2);
			deslx = chunkWidth*(j + (chuckAmount - reduction_size)/2);
			isBlack = handleImage_recursive(image, 
					x + deslx, y + desly,
					chunkHeight, chunkWidth,
					CHUNK_AMOUNT,
					IDENTIFY_BLACK_SIZE_THRESHOLD,
					reductionFactor);
			if(isBlack) return 1;
		}
	}
	return 0;
}

void ImageProcessing::handleImage(const sensor_msgs::ImageConstPtr& image)
{
	
	if(image->encoding != "rgb8")
	{
		ROS_INFO("-------------------------- NOT RGB8!");
		return;
	}

	int matrix[CHUNK_AMOUNT][CHUNK_AMOUNT];
	int deslx, desly;
	int isBlack;
	int chunkHeight = image->height/CHUNK_AMOUNT;
	int chunkWidth = image->width/CHUNK_AMOUNT;
	for(int i = 0; i < CHUNK_AMOUNT; i++)
	{
		for(int j = 0; j < CHUNK_AMOUNT; j++)
		{
			deslx = chunkWidth*j;
			desly = chunkHeight*i;
			isBlack = handleImage_recursive(image, 
					deslx, desly, 
					chunkHeight, chunkWidth, 
					CHUNK_AMOUNT, 
					IDENTIFY_BLACK_SIZE_THRESHOLD,
					IDENTIFY_BLACK_REDUCTION_FACTOR);

			if(isBlack) 	matrix[i][j] = 1;
			else 		matrix[i][j] = 0;
		}
	}

	std_msgs::Int16MultiArray msg;
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[0].label = "height";
	msg.layout.dim[0].size = CHUNK_AMOUNT;
	msg.layout.dim[0].stride = CHUNK_AMOUNT;
	msg.layout.data_offset = 0;
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[1].label = "width";
	msg.layout.dim[1].size = CHUNK_AMOUNT;
	msg.layout.dim[1].stride = 1;
	for(int i = 0; i < CHUNK_AMOUNT; i++)
	{
		for(int j = 0; j < CHUNK_AMOUNT; j++)
		{
			msg.data.push_back(matrix[i][j]);
		}
	}

	handleImagePub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processing");
        ros::NodeHandle nh;
	ImageProcessing imageProcessing(nh);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("camera/image_raw",1, &ImageProcessing::handleImage, &imageProcessing);

        ros::spin();
}
