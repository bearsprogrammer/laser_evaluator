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
        {
            //ROS_INFO("exception");
            continue;
        }
        allen::LaserPointCloud temp_lpc;

		//calibrate scale of pointcloud
        val *= scale_factor[idx];
        float angle = angle_min + angle_increment * (float)i;
        float x = cos(angle) * val;     //get x, y from vector
        float y = sin(angle) * val;
        tf::Vector3 p(x, y, 0);
        tf::Vector3 reprj_p = sensors[idx]->R * p + sensors[idx]->T;

        cv::Point2f tf_tmp_pt;
        tf_tmp_pt.x = 1000.0f * reprj_p.getX();     //m to mm
        tf_tmp_pt.y = 1000.0f * reprj_p.getY();
        temp_lpc.laser_stamp_ = msg->header.stamp;
        temp_lpc.angle = angle;
        temp_lpc.laser_coordinate_ = tf_tmp_pt;

        tmp_pointcloud.push_back(temp_lpc);

        if(std::isinf(tf_tmp_pt.x) || std::isinf(tf_tmp_pt.y))	
        {
            //ROS_INFO("exception");
            continue;
        }

        if(flag.get_flag(allen::FLAG::Name::imshow))
        {
            cv::Point2f tf_grid_pt; 
            tf_grid_pt.x = grid.robot_col - tf_tmp_pt.y * grid.mm2pixel;
            tf_grid_pt.y = grid.robot_row - tf_tmp_pt.x * grid.mm2pixel;
            //printf("before_laser-> %lf, %lf\n", tf_tmp_pt.x, tf_tmp_pt.y);
            //cv::Point2f tmp_laser_pt = grid2laser(tf_grid_pt, 0, grid);

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

    //float margin_grid = 300.0f;
    float margin_grid = GRID_MARGIN; 

    grid_global.base_pt.push_back(cv::Point2f(margin_grid, (float)grid_global.grid_row-margin_grid));
    grid_global.base_pt.push_back(cv::Point2f((float)grid_global.grid_col-margin_grid, margin_grid));
    grid_global.base_pt.push_back(cv::Point2f(margin_grid, margin_grid));
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
            //cv::Point2f tmp_pt_mm = tmp_lpc.laser_coordinate_ * 1000.0f;     //m to mm
            cv::Point2f tmp_pt_mm = tmp_lpc.laser_coordinate_;
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
    for(int i = 0; i < (int)target_.size(); i++)
    {
        if(std::isinf(target_[i].centroid_pt.x) || std::isinf(target_[i].centroid_pt.y))    continue;
        cv::Point tmp_center_pt = laser2grid(target_[i].centroid_pt, grid_global.base_pt[SRCFRAME], grid_global.mm2pixel);
        cv::circle(Canvas, tmp_center_pt, target_[i].target_radius * grid_tracker.mm2pixel, cv::Scalar(0,0,255), 2);
    }

    Globalmap = Canvas.clone();
    if(gui.initialize)  
    {
        if((int)target_.size() == TARGETNUM && flag.get_flag(allen::FLAG::Name::initTarget))
        {
            flag.set_flag_on(allen::FLAG::Name::targetOn);
            flag.set_flag_off(allen::FLAG::Name::initTarget);
            gui.clicked_button(gui.canvas, gui.b_init);
        }
        gui.display_grid(gui.canvas, Globalmap, grid_global, flag.get_flag(allen::FLAG::Name::initTarget));
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
    tmp_target.set_Centerpt();
    tmp_target.target_radius = TRACKING_RADIUS;
    init_SRC(tmp_target);

    _target.push_back(tmp_target);
    printf("[target_]-> size: %d, idx: %d, rect: [w:%d, h:%d]\n", (int)_target.size(),
            _target.back().target_idx, _target.back().target_rect.width, _target.back().target_rect.height);
    flag.set_flag_off(allen::FLAG::Name::setRect);
}
void tracker::init_SRC(allen::Target &_target)
{
    if(_target.target_idx != ROBOT_IDX)     return;

    cv::Rect tmp_rect = _target.target_rect;
    cv::Mat debug_mat(300, 300, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat tmp_canvas = gui.canvas.clone();

    for(int i = 0; i < (int)bag_cloud_.size(); i++)
    {
        bag_t tmp_bag = bag_cloud_[i];
        for(int j = 0; j < (int)tmp_bag.size(); j++)
        {
            allen::LaserPointCloud tmp_cloud = tmp_bag[j];
            cv::Point grid_pt = laser2grid(tmp_cloud.laser_coordinate_, grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);

            bool valid_pt = false;
            if(tmp_rect.contains(grid_pt))  valid_pt = true;
            if(valid_pt)
            {
                _target.src_object_pts.push_back(tmp_cloud.laser_coordinate_);
            }
        }
    }
    if(_target.src_object_pts.empty())
        ROS_ERROR("Contour of Robot is empty..");
    else
    {
        _target.src_dp_mat = tmp_canvas(tmp_rect);
        ROS_INFO("Number of Point-cloud for Robot contour is set: %d", (int)_target.src_object_pts.size());
    }

    if(!_target.src_dp_mat.empty())
    {
        cv::Mat resize_contour;
        cv::Size tmp_size(_target.src_dp_mat.cols * 2.0, _target.src_dp_mat.rows * 2.0);
        cv::resize(_target.src_dp_mat, resize_contour, tmp_size, CV_INTER_LINEAR);
        cv::imshow("contour", resize_contour);
    }
}
void tracker::tracking_Targets(std::vector<allen::Target> &_target)
{
    if((int)_target.size() != TARGETNUM)    return;

    cv::Mat tmp_debug_mat = gui.canvas.clone();
    for(int i = 0; i < (int)_target.size(); i++)
    {
        allen::Target tmp_target = _target[i];
        //grid2laser
        cv::Point grid_pt = tmp_target.center_pt;
        cv::Point2f laser_pt = grid2laser(grid_pt, grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);

        //printf("[target_%d]->[gx: %d][gy: %d]\n", i, tmp_target.center_pt.x, tmp_target.center_pt.y);

        //cv::rectangle(tmp_debug_mat, _target[i].target_rect, cv::Scalar(255,0,0), 2);
        std::vector<cv::Point2f> tmp_pts;
        cv::Point2f centroid_laser = rearrange_Centroid(grid_pt, laser_pt, bag_cloud_, tmp_debug_mat, tmp_pts);

        if(std::isinf(centroid_laser.x) || std::isinf(centroid_laser.y))     continue;
        
        //printf("centroid: [%f, %f]\n", centroid_laser.x, centroid_laser.y);

        //update
        tmp_target.object_pts = tmp_pts;
        tmp_target.centroid_pt = centroid_laser;
        tmp_target.center_pt = laser2grid(centroid_laser, grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);

        //cv::circle(tmp_debug_mat, tmp_target.center_pt, tmp_target.target_radius * grid_tracker.mm2pixel, 
                    //cv::Scalar(0,0,255), 2);

        //new centroid to target
        _target[i] = tmp_target;
    }

    match_Robot(_target);
    //std::cout << std::endl;
    //std::cout << std::endl;
    //cv::imshow("debug", tmp_debug_mat);
    //cv::waitKey(0);
}
void tracker::display_Pointcloud(cv::Mat &_src1, cv::Mat &_src2, std::string _win_name)
{
    if(!flag.get_flag(allen::FLAG::Name::imshow))   return;

    cv::Mat tmp_canvas(grid_tracker.grid_row, grid_tracker.grid_col, CV_8UC3, cv::Scalar(0,0,0));

    for(int i = 0; i < (int)_src1.rows; i++)
    {
        cv::Point2f tmp_laser;
        tmp_laser.x = _src1.at<float>(i, 0);
        tmp_laser.y = _src1.at<float>(i, 1);

        tmp_laser.x *= 1000.0f;
        tmp_laser.y *= 1000.0f;
        //std::cout << "x: " << tmp_laser.x << " y: " << tmp_laser.y << std::endl;
        cv::Point tmp_grid = laser2grid(tmp_laser, grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);
        //std::cout << "g_x: " << tmp_grid.x << " g_y: " << tmp_grid.y << std::endl;

        cv::circle(tmp_canvas, tmp_grid, 2, cv::Scalar(0,255,0), 1);
    }
    for(int i = 0; i < (int)_src2.rows; i++)
    {
        cv::Point2f tmp_laser;
        tmp_laser.x = _src2.at<float>(i, 0);
        tmp_laser.y = _src2.at<float>(i, 1);

        tmp_laser.x *= 1000.0f;
        tmp_laser.y *= 1000.0f;
        //std::cout << "x: " << tmp_laser.x << " y: " << tmp_laser.y << std::endl;
        cv::Point tmp_grid = laser2grid(tmp_laser, grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);
        //std::cout << "g_x: " << tmp_grid.x << " g_y: " << tmp_grid.y << std::endl;

        cv::circle(tmp_canvas, tmp_grid, 2, cv::Scalar(0,0,255), 1);
    }

    cv::imshow(_win_name, tmp_canvas);
}
std::vector<cv::Point2f> tracker::extract_Contour(allen::Target &_robot, std::vector<bag_t> &_bag_cloud)
{
    if((int)_robot.object_pts.size() == 0)  return std::vector<cv::Point2f>();

    cv::Mat tmp_canvas = gui.canvas.clone();
    //set search region
    cv::Rect tmp_region = _robot.set_Region(_robot.center_pt);
    cv::Mat tmp_contour = tmp_canvas(tmp_region);
    cv::Mat resize_contour;

    if(tmp_contour.cols <= 0 || tmp_contour.rows <= 0)
        return std::vector<cv::Point2f>();

    cv::resize(tmp_contour, resize_contour, cv::Size(tmp_contour.cols*2.0, tmp_contour.rows*2.0), CV_INTER_LINEAR);

    cv::imshow("dst", resize_contour);
    //cv::waitKey(0);
    std::vector<cv::Point2f> output_contour;
    output_contour.reserve((int)_robot.object_pts.size());

    for(int i = 0; i < (int)_bag_cloud.size(); i++)
    {
        bag_t tmp_cloud = _bag_cloud[i];
        for(int j = 0; j < (int)tmp_cloud.size(); j++)
        {
            allen::LaserPointCloud tmp_pt = tmp_cloud[j];
            cv::Point grid_pt = laser2grid(tmp_pt.laser_coordinate_, grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);

            bool valid_pt = false;
            if(tmp_region.contains(grid_pt))    valid_pt = true;
            if(valid_pt)
            {
                output_contour.push_back(tmp_pt.laser_coordinate_);
            }

        }
    }

    return output_contour;
}
void tracker::match_Robot(std::vector<allen::Target> &_target)
{
    if((int)_target.size() != TARGETNUM)    return;

    std::vector<cv::Point2f> tmp_src_points = _target[ROBOT_IDX].src_object_pts;
    std::vector<cv::Point2f> tmp_dst_points = extract_Contour(_target[ROBOT_IDX], bag_cloud_);
    allen::Target tmp_t;
    std::vector<cv::Point2f> src_points = tmp_t.cvtFloat(tmp_src_points);       //cvtFloat  -> mm to m
    std::vector<cv::Point2f> dst_points = tmp_t.cvtFloat(tmp_dst_points);       //cvtFloat  -> mm to m

    bool valid_run = false;
    if((int)src_points.size() != 0 && (int)dst_points.size() != 0)    valid_run = true;
    if(valid_run)
    {
        //printf("src: %d, dst: %d\n", (int)src_points.size(), (int)dst_points.size());

        cv::Mat draw, src_frame, src_frame_, dst_frame, dst_frame_;
        draw = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(255,255,255));
        src_frame = cv::Mat((int)src_points.size(), 2, CV_32FC1, src_points.data());     //from
        dst_frame = cv::Mat((int)dst_points.size(), 2, CV_32FC1, dst_points.data());     //to

        display_Pointcloud(src_frame, dst_frame, "before");
        output_robot.translate(dst_frame, dst_frame_);
        display_Pointcloud(src_frame, dst_frame_, "after");

        cv::flann::Index flann_idx(src_frame, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        allen::Frame tmp_output;
        bool success = icp.run(dst_frame, src_frame, tmp_output, flann_idx, draw);

        if(success)
        {
            output_robot = tmp_output;
            printf("output_robot: %lf, %lf, %lf\n", output_robot.x, output_robot.y, output_robot.th);
        }
    }

}
cv::Point2f tracker::rearrange_Centroid(cv::Point _grid_src, cv::Point2f _laser_src, std::vector<bag_t> &_bag_cloud, 
                                    cv::Mat &_debug_mat, std::vector<cv::Point2f> &_tmp_object_pts)
{
    if((int)grid_tracker.base_pt.size() == 0)   
    {
        ROS_ERROR("[tracker]Not enough samples to calc centroid..");
        cv::Point2f inf_pt;
        bag_t empty_bag;
        inf_pt.x = std::numeric_limits<float>::infinity();
        inf_pt.y = std::numeric_limits<float>::infinity();

        return inf_pt;
    }

    bag_t tmp_target_bag;
    cv::Point2f centroid;
    std::vector<cv::Point2f> tmp_object_pts;

    for(int i = 0; i < (int)_bag_cloud.size(); i++)
    {
        bag_t tmp_pointcloud = _bag_cloud[i];
        cv::Scalar tmp_scalar = sensors[i]->pointcolor;

        for(int j = 0; j < (int)tmp_pointcloud.size(); j++)
        {
            allen::LaserPointCloud tmp_lpc = tmp_pointcloud[j];
            float dist = get_dist2f(_laser_src, tmp_lpc.laser_coordinate_);
            if(dist <= TRACKING_RADIUS)
            {
                tmp_target_bag.push_back(tmp_lpc);
                tmp_object_pts.push_back(tmp_lpc.laser_coordinate_);

                //laser2grid
                cv::Point tmp_scan_pt = laser2grid(tmp_lpc.laser_coordinate_, grid_tracker.base_pt[SRCFRAME], 
                                                    grid_tracker.mm2pixel);
                //cv::line(_debug_mat, _grid_src, tmp_scan_pt, tmp_scalar, 1);
            }

        }
    }

    centroid = calc_Mean(tmp_target_bag);
    _tmp_object_pts.swap(tmp_object_pts);
    //cv::imshow("debug", _debug_mat);

    return centroid;
}
cv::Point2f tracker::calc_Mean(bag_t _src)
{
    if((int)_src.size() == 0)
    {
        ROS_ERROR("[tracker]Not enough samples to calc means of samples..");
        cv::Point2f inf_pt;
        inf_pt.x = std::numeric_limits<float>::infinity();
        inf_pt.y = std::numeric_limits<float>::infinity();
        return inf_pt;
    }

    cv::Point2f mean_dst;
    cv::Point2f sum_pt;
    sum_pt.x = 0.0f;
    sum_pt.y = 0.0f;

    for(int i = 0; i < (int)_src.size(); i++)
    {
        cv::Point2f tmp_pt = _src[i].laser_coordinate_;
        sum_pt.x += tmp_pt.x;
        sum_pt.y += tmp_pt.y;
    }
    mean_dst.x = sum_pt.x / (float)_src.size();
    mean_dst.y = sum_pt.y / (float)_src.size();

    return mean_dst;
}
cv::Point tracker::laser2grid(cv::Point2f _src_pt, cv::Point _base_pt, float _scale)
{
    cv::Point pt_dst;
    pt_dst.x = _base_pt.x - _src_pt.y * _scale;
    pt_dst.y = _base_pt.y - _src_pt.x * _scale;

    return pt_dst;
}
cv::Point2f tracker::grid2laser(cv::Point _src_pt, cv::Point _base_pt, float _scale)
{
    cv::Point2f pt_dst;
    pt_dst.x = (_base_pt.y - _src_pt.y) / _scale;
    pt_dst.y = (_base_pt.x - _src_pt.x) / _scale;

    return pt_dst;
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
        bool valid_reset_b = false;

        if(gui.b_init.rect.contains(m_pt))      valid_init_b = true;
        if(gui.b_calib.rect.contains(m_pt))     valid_calib_b = true;
        if(gui.b_reset.rect.contains(m_pt))     valid_reset_b = true;
        if(valid_init_b)
        {
            gui.clicked_button(_canvas, gui.b_init);
            flag.set_flag_on(allen::FLAG::Name::initTarget);    
        } 
        if(valid_calib_b)
        {
            gui.clicked_button(_canvas, gui.b_calib);
            flag.set_flag_on(allen::FLAG::Name::calibration);
        }
        if(valid_reset_b)
        {
            gui.clicked_button(_canvas, gui.b_reset);
            flag.set_flag_on(allen::FLAG::Name::reset);
        }
    }
    if(flag.get_flag(allen::FLAG::Name::initTarget))
    {
        if(m_drag && gui.mi.drag_s_pt.x != 0 && gui.mi.drag_s_pt.y != 0)
        {
            cv::Point m_drag_pt(gui.mi.getX(), gui.mi.getY());
            if(gui.d_map.rect.contains(m_drag_pt))
            {
                gui.drag_rect = cv::Rect(gui.mi.drag_s_pt, m_drag_pt);
                flag.set_flag_on(allen::FLAG::Name::setRect);
            }
        }
    }
    if(flag.get_flag(allen::FLAG::Name::setRect) && m_up)
        set_Target(target_, gui.drag_rect);
    if(flag.get_flag(allen::FLAG::Name::reset))
    {
        //reset
        target_.clear();
        flag.resetFlag();
        gui.reset(_canvas);
        output_robot = allen::Frame();
    }
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
                //cv::imshow("GlobalMap", Globalmap);         //calibrated pointcloud
                cv::imshow(gui.canvas_win, gui.canvas);     //gui
                cv::waitKey(10);
            }
            if(flag.get_flag(allen::FLAG::Name::targetOn))
            {
                tracking_Targets(target_);
            }
        }
        //GetMouseEvent(gui.canvas);
        ros::spinOnce();
        r.sleep();
    }

}

