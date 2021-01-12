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

class tracker
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_1_sub_, scan_2_sub_, scan_3_sub_, scan_4_sub_;
    bool imshow;

public:
    std::vector<sensor*> sensors;
    std::vector<float> scale_factor;
    allen::Grid_param grid;

private:
    void initSubscriber();
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx);

public:
    tracker(ros::NodeHandle &_nh) :
        nh_(_nh),
        imshow(true)
    {
        //add objects of frame
        sensors.push_back(new sensor(0, "map", "laser1_calib"));
        sensors.push_back(new sensor(1, "map", "laser2_calib"));
        sensors.push_back(new sensor(2, "map", "laser3_calib"));
        sensors.push_back(new sensor(3, "map", "laser4_calib"));
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
        for(it_sensor=sensors.begin(); it_sensor!=sensors.end(); it_sensor++)
            delete *it_sensor;
    }
};
