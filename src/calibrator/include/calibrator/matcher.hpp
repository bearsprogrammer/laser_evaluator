#include <iostream>
#include <vector> 
#include <mutex>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "calibrator/templete.hpp"
#include "calibrator/parameter.hpp"
#include "calibrator/sensor.hpp"

#define SENSORNUM 4
#define SRCFRAME 0

using bag_t = std::vector<allen::LaserPointCloud>;

class matcher
{
private:
    double degree2radian;
    bool flag_dataOn;
    bool flag_syncOn;
    ros::NodeHandle nh_;
    ros::Subscriber scan_1_sub_, scan_2_sub_, scan_3_sub_, scan_4_sub_;
    cv::RNG rng;

    bool get_tf_flag, imshow;
    tf::Matrix3x3 R;
    tf::Vector3 T;
    std::string parent_frame, child_frame;
    tf::TransformListener listener;

public:
    std::vector<bag_t*> bag_cloud_;
    allen::Grid_param grid;
    std::vector<allen::LaserPointCloud> pointcloud_;
    std::vector<sensor*> sensors;
    std::vector<allen::Frame> output_frames;
    std::vector<float> scale_factor;
    cv::Mat Globalmap;

private:
    void initSubscriber();
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx);
    void get_syncData();
    void transform(cv::Mat& from, cv::Mat& to, allen::Frame frame);
    double getPointDist(cv::Mat& data, int idx1, int idx2);
    void centroid(cv::Mat& target, cv::Mat& center);
    void match(cv::flann::Index& flann_idx, cv::Mat& from, cv::Mat& to, 
            cv::Mat& from_inlier, cv::Mat& to_inlier, double ratio, cv::Mat& draw);
    bool run(cv::Mat &from, cv::Mat &to, allen::Frame &output, cv::flann::Index &flann_idx, cv::Mat &draw);
    void getTransformation(void);
    void calibrate_Frames(std::vector<allen::Frame> &_output_frames);
    void display_Globalmap(void);

public:
    matcher(ros::NodeHandle &_nh) :
        nh_(_nh),
        imshow(true),
        flag_dataOn(false), flag_syncOn(false)
    {
        degree2radian = (double)M_PI / 180.0;
        rng = cv::RNG(cv::getTickCount());
        //add sensor
        sensors.push_back(new sensor(0, "map", "laser1"));
        sensors.push_back(new sensor(1, "map", "laser2"));
        sensors.push_back(new sensor(2, "map", "laser3"));
        sensors.push_back(new sensor(3, "map", "laser4"));

        for(int i = 0; i < SENSORNUM; i++)
            sensors[i]->pointcolor = cv::Scalar(rng.uniform(50, 255), rng.uniform(50, 255), rng.uniform(50, 255));
        
        scale_factor.push_back(0.92f);
        scale_factor.push_back(1.0f);
        scale_factor.push_back(0.975f);
        scale_factor.push_back(0.899f);
        
        Globalmap = cv::Mat(grid.grid_row, grid.grid_row, CV_8UC3, cv::Scalar(0,0,0));

        initSubscriber();
    }
    ~matcher()
    {
        std::vector<sensor*>::iterator it_sensor;
        std::vector<bag_t*>::iterator it_bag;
        for(it_sensor=sensors.begin(); it_sensor!=sensors.end(); it_sensor++)
            delete *it_sensor;
        for(it_bag=bag_cloud_.begin(); it_bag!=bag_cloud_.end(); it_bag++)
            delete *it_bag;
    }
    void runLoop();
};
