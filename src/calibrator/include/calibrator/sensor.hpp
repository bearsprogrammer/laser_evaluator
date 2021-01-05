#include <iostream>
#include <mutex>
#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "calibrator/parameter.hpp"
#include "calibrator/templete.hpp"

#ifndef SENSOR_H
#define SENSOR_H
using bag_t = std::vector<allen::LaserPointCloud>;
class sensor
{
public:
    tf::Matrix3x3 R;
    tf::Vector3 T;
    std::string parent_frame, child_frame;
    tf::TransformListener listener;
    bool get_tf_flag;
    int sensor_idx;
    cv::Mat Grid_local;
    std::mutex mtx_scan;
    //allen objects
    allen::Grid_param grid;
    bag_t pointcloud;

public:
    bool get_tf()
    {
        if(parent_frame.empty() || child_frame.empty())
        {
            ROS_ERROR("There is no frame name");	
            return 0; }

        try
        {
            tf::StampedTransform tf_msg;
            ros::Time now = ros::Time::now();
            listener.lookupTransform(parent_frame, child_frame, ros::Time(0), tf_msg);

            R = tf::Matrix3x3(tf_msg.getRotation());
            T = tf::Vector3(
                tf_msg.getOrigin().x(),
                tf_msg.getOrigin().y(), 
                tf_msg.getOrigin().z());
        }
        catch(...)
        {
            ROS_ERROR("Fail to get tf");
            return 0;
        }
        
        ROS_INFO("success to get tf[%s]", child_frame.c_str());
        return 1; 
    }
    std::vector<cv::Point2f> cvtFloat(bag_t &_src)
    {
        std::vector<cv::Point2f> output;
        for(int i = 0; i < (int)_src.size(); i++)
        {
            cv::Point2f tmp_pt = _src[i].laser_coordinate_;
            output.push_back(tmp_pt);
        }
        return output;
    }

    sensor(int _idx, std::string _parent_frame, std::string _child_frame)  :
        get_tf_flag(false),
        sensor_idx(_idx),
        parent_frame(_parent_frame), child_frame(_child_frame)
    {
        Grid_local = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(0,0,0));
    }
    ~sensor()
    {}
};

#endif