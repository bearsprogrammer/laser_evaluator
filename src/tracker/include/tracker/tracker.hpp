#include <iostream>
#include <vector> 
#include <cmath>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> 
#include <tf/tf.h>
#include <opencv2/opencv.hpp>

#include "tracker/templete.hpp"
#include "tracker/parameter.hpp"
#include "tracker/sensor.hpp"

#define SENSORNUM 4
#define SRCFRAME 1 
#define SCALEFACTOR_1 0.92f
#define SCALEFACTOR_2 1.0f
#define SCALEFACTOR_3 0.975f
#define SCALEFACTOR_4 0.899f

using bag_t = std::vector<allen::LaserPointCloud>;

class tracker
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_1_sub_, scan_2_sub_, scan_3_sub_, scan_4_sub_;
    bool imshow, flag_dataOn;
    double degree2radian, radian2degree;
    cv::RNG rng;

public:
    std::vector<sensor*> sensors;
    std::vector<float> scale_factor;
    allen::Grid_param grid;
    cv::Mat Globalmap;
    std::vector<bag_t*> bag_cloud_;

private:
    void initSubscriber(void);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx);
    void get_syncData(void);

public:
    tracker(ros::NodeHandle &_nh) :
        nh_(_nh),
        imshow(true), flag_dataOn(false)
    {
        degree2radian = (double)M_PI / 180.0;
        radian2degree = 180.0 / (double)M_PI;
        rng = cv::RNG(cv::getTickCount());
        Globalmap = cv::Mat(grid.grid_row, grid.grid_row, CV_8UC3, cv::Scalar(0,0,0));

        //add objects of frame
        sensors.push_back(new sensor(0, "map", "laser1_calib"));
        sensors.push_back(new sensor(1, "map", "laser2_calib"));
        sensors.push_back(new sensor(2, "map", "laser3_calib"));
        sensors.push_back(new sensor(3, "map", "laser4_calib"));
        for(int i = 0; i < SENSORNUM; i++)
        {
            sensors[i]->pointcolor = cv::Scalar(rng.uniform(50, 255), rng.uniform(50, 255), rng.uniform(50, 255));
            bag_cloud_.push_back(new bag_t);
        }

        //set scale factor 
        scale_factor.push_back(SCALEFACTOR_1);
        scale_factor.push_back(SCALEFACTOR_2);
        scale_factor.push_back(SCALEFACTOR_3);
        scale_factor.push_back(SCALEFACTOR_4);

        initSubscriber();
    }
    ~tracker()
    {
        std::vector<sensor*>::iterator it_sensor;
        std::vector<bag_t*>::iterator it_bag;
        for(it_sensor=sensors.begin(); it_sensor!=sensors.end(); it_sensor++)
            delete *it_sensor;
        for(it_bag=bag_cloud_.begin(); it_bag!=bag_cloud_.end(); it_bag++)
            delete *it_bag;
    }
    void display_Globalmap(void);
    void runLoop(void);

};
