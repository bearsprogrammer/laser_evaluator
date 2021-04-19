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
#include <calibrator_server/Calibrate_laser.h>

#include "calibrator_server/templete.hpp"
#include "calibrator_server/parameter.hpp"
#include "calibrator_server/sensor.hpp"

#define SENSORNUM 4
#define SRCFRAME 0
#define SCALEFACTOR_1 1.0f
#define SCALEFACTOR_2 1.0f
#define SCALEFACTOR_3 1.0f
#define SCALEFACTOR_4 1.0f

using bag_t = std::vector<allen::LaserPointCloud>;  //pointcloud

class matcher
{
private:
    double degree2radian;
    double radian2degree;
    //flag
    bool flag_dataOn;
    bool flag_matchOn;
    bool flag_calibOn;
    bool imshow;

    ros::Subscriber scan_1_sub_;
    ros::Subscriber scan_2_sub_;
    ros::Subscriber scan_3_sub_;
    ros::Subscriber scan_4_sub_;

    cv::RNG rng;
    allen::Grid_param grid;
    std::vector<allen::LaserPointCloud> pointcloud_;
    std::vector<sensor*> sensors;
    std::vector<allen::Frame> output_frames;
    std::vector<float> scale_factor;
    cv::Mat Globalmap, Globalmap_calib;
    
public:
    ros::NodeHandle nh_;
    ros::ServiceServer srv;
    tf::TransformBroadcaster br;

private:
    void initSubscriber();
    void initServer();
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx);
    void get_syncData();
    void getTransformation(void);
    bool run(cv::Mat &from, cv::Mat &to, allen::Frame &output, cv::flann::Index &flann_idx, cv::Mat &draw);
    void transform(cv::Mat& from, cv::Mat& to, allen::Frame frame);
    double getPointDist(cv::Mat& data, int idx1, int idx2);
    void centroid(cv::Mat& target, cv::Mat& center);
    void match(cv::flann::Index& flann_idx, cv::Mat& from, cv::Mat& to, 
                cv::Mat& from_inlier, cv::Mat& to_inlier, double ratio, cv::Mat& draw);
    void broadcastTF(std::vector<allen::Frame> &_frame);

public:
    matcher(ros::NodeHandle &_nh) :
        nh_(_nh),
        imshow(true),
        flag_dataOn(false), flag_matchOn(false), flag_calibOn(false)
    {
        initSubscriber();
        initServer();

        degree2radian = (double)M_PI / 180.0;
        radian2degree = 180.0 / (double)M_PI;
        rng = cv::RNG(cv::getTickCount());
        //add sensor
        sensors.push_back(new sensor(0, "map", "laser1"));
        sensors.push_back(new sensor(1, "map", "laser2"));
        sensors.push_back(new sensor(2, "map", "laser3"));
        sensors.push_back(new sensor(3, "map", "laser4"));

        for(int i = 0; i < SENSORNUM; i++)
            sensors[i]->pointcolor = cv::Scalar(rng.uniform(50, 255), rng.uniform(50, 255), rng.uniform(50, 255));

        //set scale factor 
        scale_factor.push_back(SCALEFACTOR_1);
        scale_factor.push_back(SCALEFACTOR_2);
        scale_factor.push_back(SCALEFACTOR_3);
        scale_factor.push_back(SCALEFACTOR_4);
        
        Globalmap = cv::Mat(grid.grid_row, grid.grid_row, CV_8UC3, cv::Scalar(0,0,0));
        Globalmap_calib = cv::Mat(grid.grid_row, grid.grid_row, CV_8UC3, cv::Scalar(0,0,0));
    }
    ~matcher()
    {
        std::vector<sensor*>::iterator it_sensor;
        for(it_sensor=sensors.begin(); it_sensor!=sensors.end(); it_sensor++)
            delete *it_sensor;
    }
    bool run_calib(calibrator_server::Calibrate_laser::Request &req, calibrator_server::Calibrate_laser::Response &res);

};

