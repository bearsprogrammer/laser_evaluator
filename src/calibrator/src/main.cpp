//ROS
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "calibrator/sensor.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibrator");
	ROS_INFO("%s is initiating..", ros::this_node::getName().c_str());

	ros::NodeHandle nh;

	sensor laser_1(nh, 1);
	sensor laser_2(nh, 2);
	sensor laser_3(nh, 3);
	sensor laser_4(nh, 4);


	
	ros::spin();
	return 0;
}
