//ROS
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <memory>

#define SENSORNUM 4

#include "calibrator/matcher.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibrator");
	ROS_INFO("%s is initiating..", ros::this_node::getName().c_str());

	ros::NodeHandle nh;

	matcher mc(nh);
	mc.runLoop();

	return 0;
}
