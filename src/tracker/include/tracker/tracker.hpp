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
#include "tracker/Flag.hpp"
#include "tracker/gui.hpp"

#define SENSORNUM 4
#define SRCFRAME 1 
#define SCALEFACTOR_1 0.92f
#define SCALEFACTOR_2 1.0f
#define SCALEFACTOR_3 0.975f
#define SCALEFACTOR_4 0.899f
#define TARGETNUM 2
#define TRACKING_RADIUS 500.0f
#define GUI_MARGIN 100

using bag_t = std::vector<allen::LaserPointCloud>;

class tracker
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_1_sub_, scan_2_sub_, scan_3_sub_, scan_4_sub_;
    double degree2radian, radian2degree;
    cv::RNG rng; allen::FLAG flag;
    cv::Rect drag_rect;

public:
    std::vector<sensor*> sensors;
    std::vector<float> scale_factor;
    allen::Grid_param grid;
    cv::Mat Globalmap;
    std::vector<bag_t> bag_cloud_;
    allen::GUI gui;
    std::vector<allen::Target> target_;

private:
    void initSubscriber(void);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx);
    void get_syncData(void); 
public:
    tracker(ros::NodeHandle &_nh) :
        nh_(_nh)
    {
        degree2radian = (double)M_PI / 180.0;
        radian2degree = 180.0 / (double)M_PI;
        rng = cv::RNG(cv::getTickCount());
        Globalmap = cv::Mat(grid.grid_row, grid.grid_row, CV_8UC3, cv::Scalar(0,0,0));

        cv::Size canvas_size(grid.grid_col, grid.grid_row);
        gui = allen::GUI(canvas_size, GUI_MARGIN);

        //add objects of frame 
        sensors.push_back(new sensor(0, "map", "laser1_calib"));
        sensors.push_back(new sensor(1, "map", "laser2_calib"));
        sensors.push_back(new sensor(2, "map", "laser3_calib"));
        sensors.push_back(new sensor(3, "map", "laser4_calib"));
        for(int i = 0; i < SENSORNUM; i++)
        {
            sensors[i]->pointcolor = cv::Scalar(rng.uniform(50, 255), rng.uniform(50, 255), rng.uniform(50, 255));
            bag_cloud_.push_back(bag_t());
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
        //std::vector<bag_t*>::iterator it_bag;
        for(it_sensor=sensors.begin(); it_sensor!=sensors.end(); it_sensor++)
            delete *it_sensor;
        //for(it_bag=bag_cloud_.begin(); it_bag!=bag_cloud_.end(); it_bag++)
            //delete *it_bag;
    }
    void display_Globalmap(void);
    void regist_Pointcloud(cv::Point2f _src, std::vector<bag_t> &_bag_cloud);
    float get_dist2f(cv::Point2f _pt1, cv::Point2f _pt2)
    {
        return std::sqrt(std::pow(_pt1.x - _pt2.x, 2.0f) + std::pow(_pt1.y - _pt2.y, 2.0f));
    }
    cv::Point laser2grid(cv::Point2f _laser_pt, int _gui_margin, allen::Grid_param _grid_param);
    cv::Point2f grid2laser(cv::Point _canvas_grid_pt, int _gui_margin, allen::Grid_param _grid_param);
    void GetMouseEvent(cv::Mat &_canvas);
    void set_Target(std::vector<allen::Target> &_target, cv::Rect _target_rect);
    void tracking_Targets(std::vector<allen::Target> &_target);
    void runLoop(void);

};
