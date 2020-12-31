#include <iostream>
#include <mutex>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "calibrator/templete.hpp"

typedef std::vector<allen::LaserPointCloud> bag_type;

class matcher
{
private:
    double degree2radian;
    bool flag_dataOn;
    bool flag_syncOn;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_1, scan_sub_2, scan_sub_3, scan_sub_4;

    bool get_tf_flag, imshow;
    tf::Matrix3x3 R;
    tf::Vector3 T;
    std::string parent_frame, child_frame;
    tf::TransformListener listener;

    struct Grid_param 
    {   
        const int grid_row, grid_col, robot_col, robot_row;
        float mm2pixel;
        cv::Mat occup, free;
        Grid_param() : grid_row(500), grid_col(500), robot_col(250), robot_row(250)
        {
            mm2pixel = 15.0f / 1000.0f;     //1000mm(1m) -> 50 pixel, 20mm -> 1 pixel
            occup = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
            free = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
        }
    };

public:
    std::vector<bag_type> bag_cloud_;
    Grid_param grid;
    std::vector<allen::LaserPointCloud> pointcloud_;
    cv::Mat Grid_plot1, Grid_plot2, Grid_plot3, Grid_plot4;

private:
    void initSubscriber();
    bool get_tf(void);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx);

public:
    matcher(ros::NodeHandle &_nh) :
        nh_(_nh),
        flag_dataOn(false), flag_syncOn(false)
    {
        degree2radian = (double)M_PI / 180.0;
        Grid_plot1 = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(0,0,0));
        Grid_plot2 = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(0,0,0));
        Grid_plot3 = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(0,0,0));
        Grid_plot4 = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(0,0,0));
        initSubscriber();
    }
    ~matcher()
    {
        ROS_ERROR("Destroy function of matcher..");
    }
    void add_pointcloud(bag_type &_src);
    void runLoop();
};
