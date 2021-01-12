#include "tracker/tracker.hpp"

void tracker::initSubscriber()
{
    scan_1_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_1", 10, boost::bind(&tracker::scan_callback, this, _1, 0));
    scan_2_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_2", 10, boost::bind(&tracker::scan_callback, this, _1, 1));
    scan_3_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_3", 10, boost::bind(&tracker::scan_callback, this, _1, 2));
    scan_4_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_4", 10, boost::bind(&tracker::scan_callback, this, _1, 3));
}
void tracker::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx)
{
    if(!sensors[idx]->get_tf_flag) sensors[idx]->get_tf_flag = sensors[idx]->get_tf(); 

    allen::Grid_param tmp_grid;
    cv::Mat Grid(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(125, 125, 125));		

    int size = std::min((int)msg->ranges.size(), 1440);
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_increment = msg->angle_increment;
    float range_min = (float)msg->range_min;
    float range_max = (float)msg->range_max;

    std::vector<allen::LaserPointCloud> tmp_pointcloud;
    tmp_pointcloud.reserve(size);

    for(int i = 0; i < size; i++)
    {
        float val = msg->ranges[i];
        if (val <= range_min || val >= range_max || !std::isfinite(val) || val == 0.0f)
            continue;
        allen::LaserPointCloud temp_lpc;

		//calibrate scale of pointcloud
        val *= scale_factor[idx];
        float angle = angle_min + angle_increment * (float)i;
        float x = cos(angle) * val;     //get x, y from vector
        float y = sin(angle) * val;
        tf::Vector3 p(x, y, 0);
        tf::Vector3 reprj_p = sensors[idx]->R * p + sensors[idx]->T;

        cv::Point2f tf_tmp_pt;
        tf_tmp_pt.x = 1000.0f * reprj_p.getX();
        tf_tmp_pt.y = 1000.0f * reprj_p.getY();
        temp_lpc.laser_stamp_ = msg->header.stamp;
        temp_lpc.angle = angle;
        temp_lpc.laser_coordinate_ = cv::Point2f(reprj_p.getX(), reprj_p.getY());

        tmp_pointcloud.push_back(temp_lpc);

        if(std::isinf(tf_tmp_pt.x) || std::isinf(tf_tmp_pt.y))	continue;

        if(imshow)
        {
            cv::Point2f tf_grid_pt; 
            tf_grid_pt.x = grid.robot_col - tf_tmp_pt.y * grid.mm2pixel;
            tf_grid_pt.y = grid.robot_row - tf_tmp_pt.x * grid.mm2pixel;

            cv::circle(tmp_grid.occup, tf_grid_pt, 5, cv::Scalar(255, 255, 255), -1);
            cv::line(tmp_grid.free, cv::Point2f(tmp_grid.robot_col, tmp_grid.robot_row), tf_grid_pt, cv::Scalar(255,255,255), 2);
        }
    }
    sensors[idx]->mtx_scan.lock();
    sensors[idx]->pointcloud.swap(tmp_pointcloud);
    sensors[idx]->mtx_scan.unlock();

    if(imshow)
    {
        Grid += tmp_grid.free;
        Grid -= tmp_grid.occup;
        
        //plot axes of center of the robot
        cv::line(Grid, cv::Point(0, grid.robot_row), cv::Point(grid.grid_col, grid.robot_row), cv::Scalar(0,0,255));
        cv::line(Grid, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid.grid_row), cv::Scalar(0,0,255));
        
        sensors[idx]->Grid_local = Grid.clone();
        //ROS_INFO("[%s]Grid->rows: %d", sensors[idx]->child_frame.c_str(), sensors[idx]->Grid_local.rows);
    }
}

