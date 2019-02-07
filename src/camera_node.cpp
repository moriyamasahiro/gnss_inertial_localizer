#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

ros::init (argc, argv, "img_publisher");
ros::NodeHandle nh("~");
image_transport::ImageTransport it(nh);
image_transport::Publisher image_pub = it.advertise("image", 10);
image_transport::Publisher depth_pub = it.advertise("depth", 10);

void publish_image(const sensor_msgs::ImageConstPtr img, const sensor_msgs::ImageConstPtr depth)
{
	if(img->header.seq%4 != 0){
          return;  
        }
        
        image_pub.publish(img);
        depth_pub.publish(depth);
        return;
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "img_publisher");
	ros::NodeHandle nh("~");
 
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("image", 10);
 
	cv::Mat image;
	cv::VideoCapture camera(0);
 
	if (!camera.isOpened()) {
		ROS_INFO("failed to open camera.");
		return -1;
	}
 
 
	ros::Rate looprate (10);   // capture image at 10Hz
	while(ros::ok()) {
		camera >> image;
 
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		image_pub.publish(msg);
 
		ros::spinOnce();
		looprate.sleep();
	}
 
	return 0;
}
