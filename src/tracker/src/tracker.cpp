#include "tracker/tracker.hpp"

void tracker::initSubscriber(void)
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

        //for debug
        tfScalar roll, pitch, yaw, x, y, z;
        sensors[idx]->R.getRPY(roll, pitch, yaw);
        x = sensors[idx]->T.getX();
        y = sensors[idx]->T.getY();
        z = sensors[idx]->T.getZ();
        //printf("callback[%d]-> rpy[%lf, %lf, %lf], origin_xyz[%lf, %lf, %lf]\n", idx, roll, pitch, yaw, x, y, z);

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

        if(flag.get_flag(allen::FLAG::Name::imshow))
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

    if(flag.get_flag(allen::FLAG::Name::imshow))
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
void tracker::display_Globalmap(void)
{
    if(!flag.get_flag(allen::FLAG::Name::imshow))   return;
    if((int)bag_cloud_[SRCFRAME].size() == 0)       return;

    allen::Grid_param grid_global;
    cv::Mat Canvas(grid_global.grid_row, grid_global.grid_col, CV_8UC3, cv::Scalar(0,0,0));

    float margin_grid = 300.0f;

    grid_global.base_pt.push_back(cv::Point2f(margin_grid, margin_grid));
    grid_global.base_pt.push_back(cv::Point2f(margin_grid, (float)grid_global.grid_row-margin_grid));
    grid_global.base_pt.push_back(cv::Point2f((float)grid_global.grid_col-margin_grid, margin_grid));
    grid_global.base_pt.push_back(cv::Point2f((float)grid_global.grid_col-margin_grid, (float)grid_global.grid_row-margin_grid));

    cv::Point2f tmp_base_pt = grid_global.base_pt[SRCFRAME];

	cv::Point tmp_mark_pt(50, 50);

    for(int i = 0; i < (int)bag_cloud_.size(); i++)
    {
        //bag_t tmp_pointcloud = sensors[i]->pointcloud;
        bag_t tmp_pointcloud = bag_cloud_[i];
        cv::Scalar tmp_scalar;
        if(i == SRCFRAME)
            tmp_scalar = cv::Scalar(0,255,0);   //green
        else
            tmp_scalar = sensors[i]->pointcolor;
        //cv::Point2f tmp_base_pt = grid_global.base_pt[i];
        //if(i == 0 || i == 2)    continue;

        for(int j = 0; j < (int)tmp_pointcloud.size(); j++)
        {
            //laser2grid
            allen::LaserPointCloud tmp_lpc = tmp_pointcloud[j];
            cv::Point2f tmp_pt_mm = tmp_lpc.laser_coordinate_ * 1000.0f;     //m to mm
            if(std::isinf(tmp_pt_mm.x) || std::isinf(tmp_pt_mm.y))  continue;
            cv::Point2f tmp_pt_grid;
            tmp_pt_grid.x = tmp_base_pt.x - tmp_pt_mm.y * grid_global.mm2pixel;
            tmp_pt_grid.y = tmp_base_pt.y - tmp_pt_mm.x * grid_global.mm2pixel;

            cv::circle(Canvas, tmp_pt_grid, 2, tmp_scalar, -1);
        }
		//mark output parameter
        tfScalar roll, pitch, yaw, x, y, z;
        sensors[i]->R.getRPY(roll, pitch, yaw);
        x = sensors[i]->T.getX();
        y = sensors[i]->T.getY();
        z = sensors[i]->T.getZ();

		cv::circle(Canvas, tmp_mark_pt, 10, tmp_scalar, -1);
		cv::putText(Canvas, 
				cv::format("laser%d", i+1), 
				cv::Point(tmp_mark_pt.x + 25, tmp_mark_pt.y+5), cv::FONT_HERSHEY_COMPLEX, 0.7f, tmp_scalar, 1, CV_AA);
		cv::putText(Canvas, 
			cv::format("-> t[x: %lf, y: %lf], R[th: %lf], Scale[%f]", x, y, yaw*radian2degree, scale_factor[i]), 
			cv::Point(tmp_mark_pt.x + 100, tmp_mark_pt.y+5), cv::FONT_HERSHEY_COMPLEX, 0.7f, tmp_scalar, 1, CV_AA);
		tmp_mark_pt.y += 25;
    }
    //SRC_FRAME
    //cv::line(Canvas, cv::Point(0, tmp_base_pt.y), 
                        //cv::Point(grid_global.grid_col, tmp_base_pt.y), cv::Scalar(0,0,255));
    //cv::line(Canvas, cv::Point(tmp_base_pt.x, 0), 
                        //cv::Point(tmp_base_pt.x, grid_global.grid_row), cv::Scalar(0,0,255));
    //Center
    //cv::line(Canvas, cv::Point(0, grid_global.robot_row), 
                        //cv::Point(grid_global.grid_col, grid_global.robot_row), cv::Scalar(0,0,255));
    //cv::line(Canvas, cv::Point(grid_global.robot_col, 0), 
                        //cv::Point(grid_global.robot_col, grid_global.grid_row), cv::Scalar(0,0,255));

    Globalmap = Canvas.clone();
    if(gui.initialize)  
    {
        if((int)target_.size() == TARGETNUM && flag.get_flag(allen::FLAG::Name::initTarget))
        {
            flag.set_flag_on(allen::FLAG::Name::targetOn);
            flag.set_flag_off(allen::FLAG::Name::initTarget);
            gui.clicked_button(gui.canvas, gui.b_init);
        }
        gui.display_grid(gui.canvas, Globalmap, drag_rect, grid_global, flag.get_flag(allen::FLAG::Name::initTarget));
    }
}
void tracker::set_Target(std::vector<allen::Target> &_target, cv::Rect _target_rect)
{
    if((int)_target.size() >= TARGETNUM)    return;

    allen::Target tmp_target;
    if((int)_target.size() != 0)
    {
        int _idx = _target.back().target_idx;
        tmp_target.target_idx = ++_idx;
    }
    tmp_target.target_rect = _target_rect;

    _target.push_back(tmp_target);
    printf("[target_]-> size: %d, idx: %d, rect: [w:%d, h:%d]\n", (int)_target.size(),
            _target.back().target_idx, _target.back().target_rect.width, _target.back().target_rect.height);
    flag.set_flag_off(allen::FLAG::Name::setRect);
}
void tracker::tracking_Targets(std::vector<allen::Target> &_target, std::vector<bag_t> &_bag_cloud)
{
    if((int)_target.size() != TARGETNUM)    return;

    for(int i = 0; i < (int)_target.size(); i++)
    {
        allen::Target tmp_target = _target[i];
        //grid2robot
         

    }



}
void tracker::GetMouseEvent(cv::Mat &_canvas)
{
    if(_canvas.empty())     return;
    if(!gui.initialize)     return;
    
    bool m_down = gui.mi.getDown();
    bool m_drag = gui.mi.getDrag();
    bool m_up = gui.mi.getUp();
    if(m_down)
    {
        cv::Point m_pt(gui.mi.getX(), gui.mi.getY());
        if(m_pt.x < 0 || m_pt.y < 0 || m_pt.x > gui.canvas.cols || m_pt.y > gui.canvas.rows)    return;
        //printf("m_pt[x: %d, y: %d]\n", m_pt.x, m_pt.y);

        if(gui.d_map.rect.contains(m_pt))
            gui.mi.drag_s_pt = cv::Point(gui.mi.getX(), gui.mi.getY());

        bool valid_init_b = false;
        bool valid_calib_b = false;
        if(gui.b_init.rect.contains(m_pt))      valid_init_b = true;
        if(gui.b_calib.rect.contains(m_pt))     valid_calib_b = true;
        if(valid_init_b)
        {
            gui.clicked_button(_canvas, gui.b_init);
            flag.set_flag_on(allen::FLAG::Name::initTarget);    //TODO
            //TODO 
            //make auto flag_off for initTarget
        } 
        if(valid_calib_b)
        {
            gui.clicked_button(_canvas, gui.b_calib);
            flag.set_flag_on(allen::FLAG::Name::calibration);
        }
    }
    if(flag.get_flag(allen::FLAG::Name::initTarget))
    {
        if(m_drag && gui.mi.drag_s_pt.x != 0 && gui.mi.drag_s_pt.y != 0)
        {
            cv::Point m_drag_pt(gui.mi.getX(), gui.mi.getY());
            if(gui.d_map.rect.contains(m_drag_pt))
            {
                //printf("drag_[x: %d, y: %d][x: %d, y: %d]\n", 
                        //gui.mi.drag_s_pt.x, gui.mi.drag_s_pt.y, m_drag_pt.x, m_drag_pt.y);
                drag_rect = cv::Rect(gui.mi.drag_s_pt, m_drag_pt);
                //printf("rect[tl: %d, %d][br: %d, %d]\n\n", drag_rect.tl().x, drag_rect.tl().y,
                                                            //drag_rect.br().x, drag_rect.br().y);
                //ROS_INFO("rect[tl: %d, %d][br: %d, %d]", drag_rect.tl().x, drag_rect.tl().y,
                                                            //drag_rect.br().x, drag_rect.br().y);
                flag.set_flag_on(allen::FLAG::Name::setRect);
            }
        }
    }
    if(flag.get_flag(allen::FLAG::Name::setRect) && m_up)
        set_Target(target_, drag_rect);

}
void tracker::get_syncData(void)
{
    for(int i = 0; i < (int)sensors.size(); i++)
    {
        if(sensors[i]->pointcloud.size() == 0)  
        {
            ROS_ERROR("Not enough sensor[%s] data in [tracker]", sensors[i]->child_frame.c_str());
            flag.set_flag_off(allen::FLAG::Name::dataOn);
            return;
        }
        sensors[i]->mtx_scan.lock();
        bag_cloud_[i] = sensors[i]->pointcloud;
        sensors[i]->mtx_scan.unlock();
        //printf("bag_cloud[%d]-> size: %d\n", i, (int)bag_cloud_[i]->size());
        //printf("sensors[%d]-> size: %d\n", i, (int)sensors[i]->pointcloud.size());
    }
    //std::cout<<std::endl;
    if(!flag.get_flag(allen::FLAG::Name::dataOn))   flag.set_flag_on(allen::FLAG::Name::dataOn);
}
void tracker::runLoop(void)
{
    ros::Rate r(15);
    while (ros::ok())
    {
        get_syncData();
        GetMouseEvent(gui.canvas);
        if(flag.get_flag(allen::FLAG::Name::dataOn))
        {
            if(flag.get_flag(allen::FLAG::Name::imshow))
            {
                display_Globalmap();
                cv::imshow("GlobalMap", Globalmap);         //calibrated pointcloud
                cv::imshow(gui.canvas_win, gui.canvas);     //gui
                cv::waitKey(10);
            }
            if(flag.get_flag(allen::FLAG::Name::targetOn))
            {
                tracking_Targets(target_, bag_cloud_);
            }
        }
        //GetMouseEvent(gui.canvas);
        ros::spinOnce();
        r.sleep();
    }

}

