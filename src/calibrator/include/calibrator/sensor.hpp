#include <iostream>
#include <mutex>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>

class sensor
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
	bool imshow;

    struct Grid_param 
    {   
        const int grid_row, grid_col, robot_col, robot_row;
        float mm2pixel;
        cv::Mat occup, free, occup_c, free_c;
        Grid_param() : grid_row(500), grid_col(500), robot_col(250), robot_row(350)
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
    std::vector<std::pair<float, cv::Point2f> > pointcloud;
    std::string topic_name_;
    cv::Mat Grid_plot;

public:
    sensor(ros::NodeHandle &_nh, int _idx)  :
        imshow(true), nh_(_nh)
    {
        topic_name_ = cv::format("/scan_%d", _idx);
		scan_sub_ = nh_.subscribe(topic_name_, 10, &sensor::scan_callback, this);
        Grid_plot = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(0,0,0));
    }
    sensor()
    {}
    ~sensor()
    {}
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
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
        std::vector<std::pair<float, cv::Point2f> > tmp_pointcloud;

        //printf("[%s]-> ranges size: %d\n", topic_name_.c_str(), size);
        //printf("[%s]range->[min: %f m][max: %f m]\n", topic_name_.c_str(), range_min, range_max);

        for(int i = 0; i < size; i++)
        {
			float val = msg->ranges[i];
			if (val <= range_min || val >= range_max || !std::isfinite(val) || val == 0.0f)
				continue;
            std::pair<float, cv::Point2f> tmp_pc;
			float angle = angle_min + angle_increment * (float)i;
			float x = cos(angle) * val;     //get x, y from vector
			float y = sin(angle) * val;
            tmp_pc.first = angle;
            tmp_pc.second = cv::Point2f(x, y);

            printf("[%s]angle-> %f, [x, y]-> %f, %f\n", topic_name_.c_str(), 
                                                        tmp_pc.first, tmp_pc.second.x, tmp_pc.second.y);
            tmp_pointcloud.push_back(tmp_pc);

			cv::Point2f tmp_pt_mm;
			tmp_pt_mm.x = 1000.0f * x;
			tmp_pt_mm.y = 1000.0f * y;

			if(std::isinf(tmp_pt_mm.x) || std::isinf(tmp_pt_mm.y))	continue;
			cv::Point2f grid_pt;
			grid_pt.x = grid.robot_col - tmp_pt_mm.y * grid.mm2pixel;
			grid_pt.y = grid.robot_row - tmp_pt_mm.x * grid.mm2pixel;

			if(imshow)
			{
				cv::circle(grid.occup, grid_pt, 5, cv::Scalar(255, 255, 255), -1);
				cv::line(grid.free, cv::Point2f(grid.robot_col, grid.robot_row), grid_pt, cv::Scalar(255,255,255), 2);
            }
        }

        pointcloud.swap(tmp_pointcloud);

        if(imshow)
        {
            Grid += grid.free;
            Grid -= grid.occup;
            Grid_plot = Grid.clone();
        }

    }
};

