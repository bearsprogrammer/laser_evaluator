//ROS
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "tracker/tracker.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker");
	ROS_INFO("%s is initiating..", ros::this_node::getName().c_str());

	ros::NodeHandle nh;
	tracker tk(nh);
	
	ros::spin();
	return 0;
}
