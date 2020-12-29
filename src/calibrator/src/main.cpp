//ROS
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <memory>

#define SENSORNUM 4

#include "calibrator/sensor.hpp"
#include "calibrator/matcher.hpp"

//typedef std::vector<allen::sensor> unit_sensor_t;
typedef std::vector<std::unique_ptr<allen::sensor> > unit_sensor_t;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "calibrator");
	ROS_INFO("%s is initiating..", ros::this_node::getName().c_str());

	ros::NodeHandle nh;

	//allen::sensor laser_1(nh, 1, "map", "laser1");
	//allen::sensor laser_2(nh, 2, "map", "laser2");
	//allen::sensor laser_3(nh, 3, "map", "laser3");
	//allen::sensor laser_4(nh, 4, "map", "laser4");
	unit_sensor_t psensors;
	psensors.push_back(new allen::sensor(nh, 1, "map", "laser1"));
	//sensors.push_back(laser_1);
	//sensors.push_back(laser_1);
	//sensors.push_back(laser_1);
	//sensors.push_back(allen::sensor(nh, 2, "map", "laser2"));
	//sensors.push_back(allen::sensor(nh, 3, "map", "laser3"));
	//sensors.push_back(allen::sensor(nh, 4, "map", "laser4"));

	matcher mc;

	std::thread t_matcher([&]()
	{
		mc.runLoop();
	});
	std::thread t_visualizer([&]()
	{
		ros::Rate r(15);
		while (ros::ok())
		{
			//cv::Mat tmp_tf_mat_1 = laser_1.tf_Grid_plot.clone();
			//cv::Mat tmp_tf_mat_2 = laser_2.tf_Grid_plot.clone();
			//cv::Mat tmp_tf_mat_3 = laser_3.tf_Grid_plot.clone();
			//cv::Mat tmp_tf_mat_4 = laser_4.tf_Grid_plot.clone();

			//cv::imshow("tf" + laser_1.topic_name_, tmp_tf_mat_1);
			//cv::imshow("tf" + laser_2.topic_name_, tmp_tf_mat_2);
			//cv::imshow("tf" + laser_3.topic_name_, tmp_tf_mat_3);
			//cv::imshow("tf" + laser_4.topic_name_, tmp_tf_mat_4);

			cv::waitKey(10);
			r.sleep();
		}
	});


	ros::MultiThreadedSpinner m_spin(4);	
	m_spin.spin();
	//ros::AsyncSpinner spinner(4);
	//spinner.start();
	//ros::waitForShutdown();
	//ros::spin();

	t_matcher.join();
	t_visualizer.join();

	return 0;
}
