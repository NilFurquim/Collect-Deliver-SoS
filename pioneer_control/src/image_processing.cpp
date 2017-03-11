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
	handle_image_pub = n.advertise<std_msgs::Int16MultiArray>(IMAGE_PROCESSED_TOPIC, 100);
}

int ImageProcessing::handleImage_recursive(
		const sensor_msgs::ImageConstPtr& image, 
		int x, int y, int height, int width, 
		int chunk_amount, int size_threshold,
		float reduction_factor)
{
	if(height <= size_threshold || width <= size_threshold)
	{
		int mid = (y + height/2) * image->step + (x + width/2)*3;
		uint8_t mid_red = image->data[mid];
		uint8_t mid_green = image->data[mid + 1];
		uint8_t mid_blue = image->data[mid + 2];

		if(mid_red < 6 && mid_green < 6 && mid_blue < 6)
			return 1;
		return 0;
	}

	int desl_x, desl_y;
	int chunk_height = height/chunk_amount;
	int chunk_width = width/chunk_amount;
	int is_black = 0;
	int count = 0;

	int reduction_size = chunk_amount/reduction_factor;
	for(int i = 0; i < reduction_size; i++)
	{
		for(int j = 0; j < reduction_size; j++)
		{
			desl_y = chunk_height*(i + (chunk_amount - reduction_size)/2);
			desl_x = chunk_width*(j + (chunk_amount - reduction_size)/2);
			is_black = handleImage_recursive(image, 
					x + desl_x, y + desl_y,
					chunk_height, chunk_width,
					CHUNK_AMOUNT,
					IDENTIFY_BLACK_SIZE_THRESHOLD,
					reduction_factor);
			if(is_black) return 1;
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
	int desl_x, desl_y;
	int is_black;
	int chunk_height = image->height/CHUNK_AMOUNT;
	int chunk_width = image->width/CHUNK_AMOUNT;
	for(int i = 0; i < CHUNK_AMOUNT; i++)
	{
		for(int j = 0; j < CHUNK_AMOUNT; j++)
		{
			desl_x = chunk_width*j;
			desl_y = chunk_height*i;
			is_black = handleImage_recursive(image, 
					desl_x, desl_y, 
					chunk_height, chunk_width, 
					CHUNK_AMOUNT, 
					IDENTIFY_BLACK_SIZE_THRESHOLD,
					IDENTIFY_BLACK_REDUCTION_FACTOR);

			if(is_black) 	matrix[i][j] = 1;
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

	handle_image_pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processing");
        ros::NodeHandle nh;
	ImageProcessing imageProcessing(nh);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("camera/image_raw",1000, &ImageProcessing::handleImage, &imageProcessing);

        ros::spin();
        printf("Im out! No error.");
}
