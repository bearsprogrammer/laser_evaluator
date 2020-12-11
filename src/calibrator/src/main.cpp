//ROS
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibrator");
	ROS_INFO("%s is initiating..", ros::this_node::getName().c_str());

	ros::NodeHandle nh;
	
	ros::spin();
	return 0;
}
