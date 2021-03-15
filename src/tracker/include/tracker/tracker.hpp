#include <iostream>
#include <vector> 
#include <cmath>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> 
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <cona_msgs/GoalPose.h>

#include "tracker/parameter.hpp"
#include "tracker/templete.hpp"
#include "tracker/sensor.hpp"
#include "tracker/Flag.hpp"
#include "tracker/gui.hpp"
#include "tracker/ICP.hpp" 

#define SENSORNUM 4
#define SRCFRAME 0
#define SCALEFACTOR_1 1.0f
#define SCALEFACTOR_2 1.0f
#define SCALEFACTOR_3 1.0f
#define SCALEFACTOR_4 1.0f
#define TRACKING_RADIUS 300.0f
#define GUI_MARGIN 100
#define GRID_MARGIN 300.0f
#define TARGETNUM 2 
#define ROBOT_IDX 0
#define TARGET_IDX 1

using bag_t = std::vector<allen::LaserPointCloud>;

class tracker
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_1_sub_, scan_2_sub_, scan_3_sub_, scan_4_sub_, goalpose_sub_;
    double degree2radian, radian2degree;
    cv::RNG rng; 
    allen::FLAG flag;
    float margin_grid_tracker;
    std::mutex predict_mtx;

public:
    std::vector<sensor*> sensors;
    std::vector<float> scale_factor;
    allen::Grid_param grid, grid_tracker, grid_global;
    cv::Mat Globalmap;
    std::vector<bag_t> bag_cloud_;
    allen::GUI gui;
    std::vector<allen::Target> target_;
    ICP icp;
    allen::Frame output_matching, output_robot, predict_posi;
    allen::Target output_predict;

private:
    void initSubscriber(void);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx);
    void GoalPose_callback(const cona_msgs::GoalPose::ConstPtr &msg);
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

        margin_grid_tracker = GRID_MARGIN + GUI_MARGIN;
        grid_tracker.base_pt.push_back(cv::Point2f(margin_grid_tracker, (float)gui.canvas_s.height - margin_grid_tracker));
        grid_tracker.base_pt.push_back(cv::Point2f((float)gui.canvas_s.width - margin_grid_tracker, margin_grid_tracker));
        grid_tracker.base_pt.push_back(cv::Point2f(margin_grid_tracker, margin_grid_tracker));
        grid_tracker.base_pt.push_back(cv::Point2f((float)gui.canvas_s.width - margin_grid_tracker, 
                                                    (float)gui.canvas_s.height - margin_grid_tracker));

        grid_global.base_pt.push_back(cv::Point2f(GRID_MARGIN, (float)grid_global.grid_row-GRID_MARGIN));
        grid_global.base_pt.push_back(cv::Point2f((float)grid_global.grid_col-GRID_MARGIN, GRID_MARGIN));
        grid_global.base_pt.push_back(cv::Point2f(GRID_MARGIN, GRID_MARGIN));
        grid_global.base_pt.push_back(cv::Point2f((float)grid_global.grid_col-GRID_MARGIN, (float)grid_global.grid_row-GRID_MARGIN));

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
    void match_Robot(std::vector<allen::Target> &_target);
    cv::Point2f rearrange_Centroid(cv::Point _grid_src, cv::Point2f _laser_src, std::vector<bag_t> &_bag_cloud, 
                                        cv::Mat &_debug_mat, std::vector<cv::Point2f> &_tmp_object_pts);
    allen::Frame get_RobotPose(allen::Frame _icp_pose);
    std::vector<cv::Point2f> get_dstContour(allen::Frame _robot_pose, cv::Mat &_dst_inlier);
    cv::Point2f calc_Mean(bag_t _src);                                              //TODO: make to templete for tool library
    float get_dist2f(cv::Point2f _pt1, cv::Point2f _pt2)                            //TODO: make to templete for tool library
    {
        return std::sqrt(std::pow(_pt1.x - _pt2.x, 2.0f) + std::pow(_pt1.y - _pt2.y, 2.0f));
    }
    cv::Point laser2grid(cv::Point2f _src_pt, cv::Point _base_pt, float _scale);    //TODO: make to templete for tool library
    cv::Point2f grid2laser(cv::Point _src_pt, cv::Point _base_pt, float _scale);    //TODO: make to templete for tool library
    void GetMouseEvent(cv::Mat &_canvas);
    void set_Target(std::vector<allen::Target> &_target, cv::Rect _target_rect);
    void init_SRC(allen::Target &_target);
    void get_ZeroMeaned(std::vector<cv::Point2f> &_pointcloud, cv::Point2f _mean_pt);
    void tracking_Targets(std::vector<allen::Target> &_target);
    void display_Pointcloud(cv::Mat &_src1, cv::Mat &_src2, std::string _win_name);
    std::vector<cv::Point2f> extract_Contour(allen::Target &_robot, std::vector<bag_t> &_bag_cloud, bool _flag);
    void get_PredictCoordinate(allen::Frame _predict_posi, allen::Frame _output_robot);
    void runLoop(void);

};
