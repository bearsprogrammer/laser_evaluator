//ROS
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "sensors/sensor.hpp"

#define SENSORNUM 4

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sensors");
	ROS_INFO("%s is initiating..", ros::this_node::getName().c_str());

	ros::NodeHandle nh;
	allen::sensor l1(nh, 1, "map", "laser1");
	allen::sensor l2(nh, 2, "map", "laser2");
	allen::sensor l3(nh, 3, "map", "laser3");
	allen::sensor l4(nh, 4, "map", "laser4");
	
	std::thread t_visualizer([&]()
	{
		while (ros::ok())
		{
			cv::imshow("1", l1.tf_Grid_plot);
			cv::imshow("2", l2.tf_Grid_plot);
			cv::imshow("3", l3.tf_Grid_plot);
			cv::imshow("4", l4.tf_Grid_plot);
			cv::waitKey(10);
		}
			
	});
	ros::MultiThreadedSpinner m_spinner(4);	
	m_spinner.spin();
	t_visualizer.join();

	return 0;
}
