#include <iostream>
#include <mutex>
#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//class LaserPointCloud;

class matcher
{
private:
    double degree2radian;

public:
    //LaserPointCloud cloud_bag_;
    std::vector<cv::Mat> cloud_bag_;

public:
    matcher(std::vector<cv::Mat> &_src, int _sensor_num)
    {
        degree2radian = (double)M_PI / 180.0;
        cloud_bag_ = _src;

        if(cloud_bag_.back().empty() || (int)cloud_bag_.size() != _sensor_num)
        {
            ROS_ERROR("Size of cloud_bag is diffrent to sensor_num..");
            return;
        }
        else
            ROS_INFO("[matcher]cloud_bag size: %d", (int)cloud_bag_.size());
    }
    ~matcher()
    {
        ROS_ERROR("Destroy function of matcher..");
    }

};

