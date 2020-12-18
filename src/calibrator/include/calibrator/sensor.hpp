#include <iostream>
#include <mutex>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>

class sensor
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    std::string topic_name_;

    struct Grid_param 
    {   
        const int grid_row, grid_col, robot_col, robot_row;
        float mm2pixel;
        cv::Mat occup, free, occup_c, free_c;
        Grid_param() : grid_row(500), grid_col(500), robot_col(250), robot_row(350)
        {
            mm2pixel = 50.0f / 1000.0f;     //1000mm(1m) -> 50 pixel, 20mm -> 1 pixel
            occup = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
            free = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
            occup_c = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
            free_c = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
        }
    };

public:
    sensor(ros::NodeHandle &_nh, int _idx)
    {
        topic_name_ = cv::format("/scan_%d", _idx);
		scan_sub_ = nh_.subscribe(topic_name_, 10, &sensor::scan_callback, this);
    }
    ~sensor()
    {}
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {

    }
};

