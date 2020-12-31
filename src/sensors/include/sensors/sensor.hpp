#include <iostream>
#include <mutex>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "sensors/templete.hpp"

#ifndef SENSOR_H
#define SENSOR_H
namespace allen
{
    class sensor
    {
    private:
        //ros
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;

        bool get_tf_flag, imshow;
        tf::Matrix3x3 R;
        tf::Vector3 T;
        std::string parent_frame, child_frame;
        tf::TransformListener listener;

        struct Grid_param 
        {   
            const int grid_row, grid_col, robot_col, robot_row;
            float mm2pixel;
            cv::Mat occup, free, occup_c, free_c;
            Grid_param() : grid_row(500), grid_col(500), robot_col(250), robot_row(250)
            {
                mm2pixel = 15.0f / 1000.0f;     //1000mm(1m) -> 50 pixel, 20mm -> 1 pixel
                occup = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
                free = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
                occup_c = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
                free_c = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
            }
        };

    public:
        Grid_param grid;
        std::vector<LaserPointCloud> pointcloud_;
        std::string topic_name_;
        cv::Mat Grid_plot, tf_Grid_plot;

    private:
        bool get_tf(void)
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
                    tf_msg.getOrigin().y(), tf_msg.getOrigin().z());
            }
            catch(...)
            {
                ROS_ERROR("Fail to get tf");
                return 0;
            }
            
            ROS_INFO("success to get tf");
            return 1; 
        }

    public:
        sensor(ros::NodeHandle &_nh, int _idx, std::string _parent_frame, std::string _child_frame)  :
            get_tf_flag(false), parent_frame(_parent_frame), child_frame(_child_frame), 
            imshow(true), nh_(_nh)
        {
            topic_name_ = cv::format("/scan_%d", _idx);
            scan_sub_ = nh_.subscribe(topic_name_, 10, &sensor::scan_callback, this);

            Grid_plot = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(0,0,0));
            tf_Grid_plot = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(0,0,0));
        }
        ~sensor()
        {}
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
        {
            if(!get_tf_flag)
                get_tf_flag = get_tf();

            cv::Mat Grid(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(125, 125, 125));		
            cv::Mat Grid_clear(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(125, 125, 125));		
            if(imshow)
            {
                grid.occup.setTo(0);
                grid.free.setTo(0);
                grid.occup_c.setTo(0);
                grid.free_c.setTo(0);
            }
            int size = std::min((int)msg->ranges.size(), 1440);
            float angle_min = msg->angle_min;
            float angle_max = msg->angle_max;
            float angle_increment = msg->angle_increment;
            float range_min = (float)msg->range_min;
            float range_max = (float)msg->range_max;

            std::vector<LaserPointCloud> tmp_pointcloud;
            tmp_pointcloud.reserve(size);

            //printf("[%s]-> ranges size: %d\n", topic_name_.c_str(), size);
            //printf("[%s]range->[min: %f m][max: %f m]\n", topic_name_.c_str(), range_min, range_max);

            for(int i = 0; i < size; i++)
            {
                float val = msg->ranges[i];
                if (val <= range_min || val >= range_max || !std::isfinite(val) || val == 0.0f)
                    continue;
                LaserPointCloud temp_lpc;

                float angle = angle_min + angle_increment * (float)i;
                float x = cos(angle) * val;     //get x, y from vector
                float y = sin(angle) * val;
                tf::Vector3 p(x, y, 0);
                tf::Vector3 reprj_p = R * p + T;

                cv::Point2f tf_tmp_pt;
                tf_tmp_pt.x = 1000.0f * reprj_p.getX();
                tf_tmp_pt.y = 1000.0f * reprj_p.getY();
                temp_lpc.laser_stamp_ = msg->header.stamp;
                temp_lpc.angle = angle;
                temp_lpc.laser_coordinate_ = cv::Point2f(reprj_p.getX(), reprj_p.getY());

                //printf("[%s][stamp]-> %d, [angle]-> %f, [x, y]-> %f, %f\n", topic_name_.c_str(), 
                //temp_lpc.laser_stamp_.sec, temp_lpc.angle, temp_lpc.laser_coordinate_.x, temp_lpc.laser_coordinate_.y);
                //printf("[%s][stamp]-> %d, [angle]-> %f\n", topic_name_.c_str(), temp_lpc.laser_stamp_.sec, temp_lpc.angle);

                tmp_pointcloud.push_back(temp_lpc);

                cv::Point2f tmp_pt_mm;
                tmp_pt_mm.x = 1000.0f * x;
                tmp_pt_mm.y = 1000.0f * y;

                if(std::isinf(tmp_pt_mm.x) || std::isinf(tmp_pt_mm.y))	continue;
                if(std::isinf(tf_tmp_pt.x) || std::isinf(tf_tmp_pt.y))	continue;

                cv::Point2f grid_pt;
                grid_pt.x = grid.robot_col - tmp_pt_mm.y * grid.mm2pixel;
                grid_pt.y = grid.robot_row - tmp_pt_mm.x * grid.mm2pixel;

                cv::Point2f tf_grid_pt;
                tf_grid_pt.x = grid.robot_col - tf_tmp_pt.y * grid.mm2pixel;
                tf_grid_pt.y = grid.robot_row - tf_tmp_pt.x * grid.mm2pixel;

                if(imshow)
                {
                    cv::circle(grid.occup, grid_pt, 5, cv::Scalar(255, 255, 255), -1);
                    cv::line(grid.free, cv::Point2f(grid.robot_col, grid.robot_row), grid_pt, cv::Scalar(255,255,255), 2);
                    cv::circle(grid.occup_c, tf_grid_pt, 5, cv::Scalar(255, 255, 255), -1);
                    cv::line(grid.free_c, cv::Point2f(grid.robot_col, grid.robot_row), tf_grid_pt, cv::Scalar(255,255,255), 2);
                }
            }

            pointcloud_.swap(tmp_pointcloud);

            if(imshow)
            {
                Grid += grid.free;
                Grid -= grid.occup;
                Grid_clear += grid.free_c;
                Grid_clear -= grid.occup_c;
                
                //plot axes of center of the robot
                cv::line(Grid, cv::Point(0, grid.robot_row), cv::Point(grid.grid_col, grid.robot_row), cv::Scalar(0,0,255));
                cv::line(Grid, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid.grid_row), cv::Scalar(0,0,255));
                cv::line(Grid_clear, cv::Point(0, grid.robot_row), cv::Point(grid.grid_col, grid.robot_row), cv::Scalar(0,0,255));
                cv::line(Grid_clear, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid.grid_row), cv::Scalar(0,0,255));

                Grid_plot = Grid.clone();
                tf_Grid_plot = Grid_clear.clone();
            }

        }
    };
}

#endif