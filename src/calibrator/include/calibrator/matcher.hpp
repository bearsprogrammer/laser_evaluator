#include <iostream>
#include <mutex>
#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "calibrator/templete.hpp"
#include "calibrator/sensor.hpp"

typedef std::vector<allen::LaserPointCloud> bag_t;
class matcher
{
private:
    double degree2radian;
    bool flag_dataOn;
public:
    std::vector<bag_t> bag_cloud_;

private:
    bool checkSync(std::vector<bag_t> &_cloud);

public:
    matcher() :
        flag_dataOn(false)
    {
        degree2radian = (double)M_PI / 180.0;
    }
    ~matcher()
    {
        ROS_ERROR("Destroy function of matcher..");
    }
    void add_pointcloud(bag_t &_src);
    void runLoop();
};

