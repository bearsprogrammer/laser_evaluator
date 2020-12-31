#include "calibrator/matcher.hpp"

void matcher::initSubscriber()
{
    scan_sub_1 = nh_.subscribe("/scan_1", 10, boost::bind(scan_callback1, _1, 1));
}
bool matcher::get_tf(void)
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
void matcher::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx)
{
    if(!get_tf_flag)
        get_tf_flag = get_tf();

    cv::Mat Grid(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(125, 125, 125));		
    if(imshow)
    {
        grid.occup.setTo(0);
        grid.free.setTo(0);
    }
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

        tmp_pointcloud.push_back(temp_lpc);

        if(std::isinf(tf_tmp_pt.x) || std::isinf(tf_tmp_pt.y))	continue;

        cv::Point2f tf_grid_pt;
        tf_grid_pt.x = grid.robot_col - tf_tmp_pt.y * grid.mm2pixel;
        tf_grid_pt.y = grid.robot_row - tf_tmp_pt.x * grid.mm2pixel;

        if(imshow)
        {
            cv::circle(grid.occup, tf_grid_pt, 5, cv::Scalar(255, 255, 255), -1);
            cv::line(grid.free, cv::Point2f(grid.robot_col, grid.robot_row), tf_grid_pt, cv::Scalar(255,255,255), 2);
        }
    }

    //pointcloud_.swap(tmp_pointcloud);

    if(imshow)
    {
        Grid += grid.free;
        Grid -= grid.occup;
        
        //plot axes of center of the robot
        cv::line(Grid, cv::Point(0, grid.robot_row), cv::Point(grid.grid_col, grid.robot_row), cv::Scalar(0,0,255));
        cv::line(Grid, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid.grid_row), cv::Scalar(0,0,255));
        
        Grid_plot1 = Grid.clone();
    }
}
void matcher::scan_callback2(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if(!get_tf_flag)
        get_tf_flag = get_tf();

    cv::Mat Grid(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(125, 125, 125));		
    if(imshow)
    {
        grid.occup.setTo(0);
        grid.free.setTo(0);
    }
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

        tmp_pointcloud.push_back(temp_lpc);

        if(std::isinf(tf_tmp_pt.x) || std::isinf(tf_tmp_pt.y))	continue;

        cv::Point2f tf_grid_pt;
        tf_grid_pt.x = grid.robot_col - tf_tmp_pt.y * grid.mm2pixel;
        tf_grid_pt.y = grid.robot_row - tf_tmp_pt.x * grid.mm2pixel;

        if(imshow)
        {
            cv::circle(grid.occup, tf_grid_pt, 5, cv::Scalar(255, 255, 255), -1);
            cv::line(grid.free, cv::Point2f(grid.robot_col, grid.robot_row), tf_grid_pt, cv::Scalar(255,255,255), 2);
        }
    }

    //pointcloud_.swap(tmp_pointcloud);

    if(imshow)
    {
        Grid += grid.free;
        Grid -= grid.occup;
        
        //plot axes of center of the robot
        cv::line(Grid, cv::Point(0, grid.robot_row), cv::Point(grid.grid_col, grid.robot_row), cv::Scalar(0,0,255));
        cv::line(Grid, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid.grid_row), cv::Scalar(0,0,255));
        
        Grid_plot2 = Grid.clone();
    }
}
void matcher::scan_callback3(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if(!get_tf_flag)
        get_tf_flag = get_tf();

    cv::Mat Grid(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(125, 125, 125));		
    if(imshow)
    {
        grid.occup.setTo(0);
        grid.free.setTo(0);
    }
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

        tmp_pointcloud.push_back(temp_lpc);

        if(std::isinf(tf_tmp_pt.x) || std::isinf(tf_tmp_pt.y))	continue;

        cv::Point2f tf_grid_pt;
        tf_grid_pt.x = grid.robot_col - tf_tmp_pt.y * grid.mm2pixel;
        tf_grid_pt.y = grid.robot_row - tf_tmp_pt.x * grid.mm2pixel;

        if(imshow)
        {
            cv::circle(grid.occup, tf_grid_pt, 5, cv::Scalar(255, 255, 255), -1);
            cv::line(grid.free, cv::Point2f(grid.robot_col, grid.robot_row), tf_grid_pt, cv::Scalar(255,255,255), 2);
        }
    }

    //pointcloud_.swap(tmp_pointcloud);

    if(imshow)
    {
        Grid += grid.free;
        Grid -= grid.occup;
        
        //plot axes of center of the robot
        cv::line(Grid, cv::Point(0, grid.robot_row), cv::Point(grid.grid_col, grid.robot_row), cv::Scalar(0,0,255));
        cv::line(Grid, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid.grid_row), cv::Scalar(0,0,255));
        
        Grid_plot3 = Grid.clone();
    }
}
void matcher::scan_callback4(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if(!get_tf_flag)
        get_tf_flag = get_tf();

    cv::Mat Grid(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(125, 125, 125));		
    if(imshow)
    {
        grid.occup.setTo(0);
        grid.free.setTo(0);
    }
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

        tmp_pointcloud.push_back(temp_lpc);

        if(std::isinf(tf_tmp_pt.x) || std::isinf(tf_tmp_pt.y))	continue;

        cv::Point2f tf_grid_pt;
        tf_grid_pt.x = grid.robot_col - tf_tmp_pt.y * grid.mm2pixel;
        tf_grid_pt.y = grid.robot_row - tf_tmp_pt.x * grid.mm2pixel;

        if(imshow)
        {
            cv::circle(grid.occup, tf_grid_pt, 5, cv::Scalar(255, 255, 255), -1);
            cv::line(grid.free, cv::Point2f(grid.robot_col, grid.robot_row), tf_grid_pt, cv::Scalar(255,255,255), 2);
        }
    }

    //pointcloud_.swap(tmp_pointcloud);

    if(imshow)
    {
        Grid += grid.free;
        Grid -= grid.occup;
        
        //plot axes of center of the robot
        cv::line(Grid, cv::Point(0, grid.robot_row), cv::Point(grid.grid_col, grid.robot_row), cv::Scalar(0,0,255));
        cv::line(Grid, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid.grid_row), cv::Scalar(0,0,255));
        
        Grid_plot4 = Grid.clone();
    }
}
void matcher::runLoop()
{
    ros::Rate r(15);
    while (ros::ok())
    {
        if(flag_dataOn && flag_syncOn)
        {

        }
        cv::imshow("grid1", Grid_plot1);
        cv::imshow("grid2", Grid_plot2);
        cv::imshow("grid3", Grid_plot3);
        cv::imshow("grid4", Grid_plot4);
        cv::waitKey(10);

        ros::spinOnce();
        r.sleep();
    }
    
}