//ROS
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <calibrator_server/Calibrate_laser.h>
#include "calibrator_server/matcher.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibrator_server");
	ROS_INFO("%s is initiating..", ros::this_node::getName().c_str());

	ros::NodeHandle nh;
	matcher calibrator(nh);
	
	ros::spin();
	return 0;
}
