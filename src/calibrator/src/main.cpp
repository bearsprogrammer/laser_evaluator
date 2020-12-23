//ROS
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "calibrator/sensor.hpp"
#include <thread>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibrator");
	ROS_INFO("%s is initiating..", ros::this_node::getName().c_str());

	ros::NodeHandle nh;

	sensor laser_1(nh, 1);
	sensor laser_2(nh, 2);
	sensor laser_3(nh, 3);
	sensor laser_4(nh, 4);

//	std::thread t_1(thread_plot, &laser_1);
//	std::thread t_1(thread_plot, [&](){});
	std::thread t_1([&]()
	{
		while (ros::ok())
		{
			cv::Mat tmp_mat_1 = laser_1.Grid_plot.clone();
			cv::Mat tmp_mat_2 = laser_2.Grid_plot.clone();
			cv::Mat tmp_mat_3 = laser_3.Grid_plot.clone();
			cv::Mat tmp_mat_4 = laser_4.Grid_plot.clone();
			cv::imshow(laser_1.topic_name_, tmp_mat_1);
			cv::imshow(laser_2.topic_name_, tmp_mat_2);
			cv::imshow(laser_3.topic_name_, tmp_mat_3);
			cv::imshow(laser_4.topic_name_, tmp_mat_4);
			cv::waitKey(10);
		}
	});

	//ros::MultiThreadedSpinner m_spin(4);	
	//m_spin.spin();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::waitForShutdown();
	//ros::spin();

	t_1.join();

	return 0;
}
