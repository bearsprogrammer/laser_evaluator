#include "tracker/tracker.hpp"

void tracker::initSubscriber(void)
{
    scan_1_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_1", 10, boost::bind(&tracker::scan_callback, this, _1, 0));
    scan_2_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_2", 10, boost::bind(&tracker::scan_callback, this, _1, 1));
    scan_3_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_3", 10, boost::bind(&tracker::scan_callback, this, _1, 2));
    scan_4_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_4", 10, boost::bind(&tracker::scan_callback, this, _1, 3));
    goalpose_sub_ = nh_.subscribe<cona_msgs::GoalPose>("/goalpose", 1, &tracker::GoalPose_callback, this);
}
void tracker::initServiceClient(void)
{
    client_calib = nh_.serviceClient<calibrator_server::Calibrate_laser>("/calibration/execute");
}
void tracker::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx)
{
    if(!sensors[idx]->get_tf_flag) sensors[idx]->get_tf_flag = sensors[idx]->get_tf(); 

    allen::Grid_param tmp_grid;
    cv::Mat Grid(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(125, 125, 125));		

	//int max_resolution = 2500;
    //int size = std::min((int)msg->ranges.size(), max_resolution);
    int size = (int)msg->ranges.size();
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
void tracker::GoalPose_callback(const cona_msgs::GoalPose::ConstPtr &msg)
{
    predict_mtx.lock();
    if(!flag.get_flag(allen::FLAG::Name::predictOn))    flag.set_flag_on(allen::FLAG::Name::predictOn);

    predict_posi_.x = msg->target_x_mm_from_robot;
    predict_posi_.y = msg->target_y_mm_from_robot;
    predict_posi_.th = 0.0;
    //ROS_INFO("Callback GoalPose[tracker]-> x: %f, y: %f, th: %f\n", msg->target_x_mm_from_robot, msg->target_y_mm_from_robot, 
                                                                        //msg->target_degree_from_robot);
    predict_mtx.unlock();
}
void tracker::display_Globalmap(void)
{
    if(!flag.get_flag(allen::FLAG::Name::imshow))   return;
    if((int)bag_cloud_[SRCFRAME].size() == 0)       return;

    cv::Mat Canvas(grid_global.grid_row, grid_global.grid_col, CV_8UC3, cv::Scalar(0,0,0));

    cv::Point2f tmp_base_pt = grid_global.base_pt[SRCFRAME];
    //cv::Point2f tmp_base_pt; 
    //tmp_base_pt.x = grid_global.grid_col / 2.0f;
    //tmp_base_pt.y = grid_global.grid_row / 2.0f;

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

    for(int i = 0; i < (int)target_.size(); i++)
    {
        if(std::isinf(target_[i].centroid_pt.x) || std::isinf(target_[i].centroid_pt.y))    continue;
        //Plot each of target
        cv::Point tmp_center_pt = laser2grid(target_[i].centroid_pt, grid_global.base_pt[SRCFRAME], grid_global.mm2pixel);
        if(i == ROBOT_IDX)
        {
            if(flag.get_flag(allen::FLAG::Name::robotOn))
            {
                //display robot contour from icp result
                cv::Point tmp_robot_pt = laser2grid(cv::Point2f(output_robot.x*1000.0f, output_robot.y*1000.0f), 
                                                    grid_global.base_pt[SRCFRAME], grid_global.mm2pixel);

                //plot axis of robot
                std::pair<allen::Frame, allen::Frame> tmp_axis_world = output_robot.getRobotAxis(1.0f);
                cv::Point axis_X_pt = laser2grid(cv::Point2f(tmp_axis_world.first.x*1000.0f, tmp_axis_world.first.y*1000.0f),
                                                    grid_global.base_pt[SRCFRAME], grid_global.mm2pixel);
                cv::Point axis_Y_pt = laser2grid(cv::Point2f(tmp_axis_world.second.x*1000.0f, tmp_axis_world.second.y*1000.0f),
                                                    grid_global.base_pt[SRCFRAME], grid_global.mm2pixel);

                cv::line(Canvas, tmp_robot_pt, axis_X_pt, cv::Scalar(0,0,255), 2);
                cv::line(Canvas, tmp_robot_pt, axis_Y_pt, cv::Scalar(255,0,0), 2);

                cv::circle(Canvas, tmp_robot_pt, 10, cv::Scalar(0,255,0), 2);   //ICP
                //cv::circle(Canvas, tmp_center_pt, target_[i].target_radius * grid_global.mm2pixel, 
                                                                            //cv::Scalar(0,0,255), 2); //tracking
                cv::putText(Canvas, 
                    cv::format("ROBOT[world]-> [x: %f][y: %f][th: %f]", output_robot.x, output_robot.y, output_robot.th), 
                    cv::Point(tmp_mark_pt.x, tmp_mark_pt.y+15), cv::FONT_HERSHEY_COMPLEX, 0.8f, cv::Scalar(250,0,0), 1, CV_AA);
            }
        }
        else if(i == TARGET_IDX)    //GroundTruth
        {
            cv::circle(Canvas, tmp_center_pt, target_[i].target_radius * grid_global.mm2pixel, cv::Scalar(0,255,0), 2);
            cv::putText(Canvas, 
                cv::format("TARGET[world]-> [x: %f][y: %f]", target_[i].centroid_pt.x/1000.0f, target_[i].centroid_pt.y/1000.0f), 
                cv::Point(tmp_mark_pt.x, tmp_mark_pt.y+15), cv::FONT_HERSHEY_COMPLEX, 0.8f, cv::Scalar(0,255,0), 1, CV_AA);
        }
        else                        //non-target
            cv::circle(Canvas, tmp_center_pt, target_[i].target_radius * grid_global.mm2pixel, cv::Scalar(255,0,0), 2);
        tmp_mark_pt.y += 30;
    }
    if(flag.get_flag(allen::FLAG::Name::predictOn) && 
        !std::isinf(eval_output.output_predict.centroid_pt.x) && !std::isinf(eval_output.output_predict.centroid_pt.y) &&
        !std::isinf(eval_output.error_centroid))
    {
        //Prediction of target from PF
        cv::putText(Canvas, 
            cv::format("PREDCITION[world]-> [x: %f][y: %f]", eval_output.output_predict.centroid_pt.x/1000.0f, 
                                                             eval_output.output_predict.centroid_pt.y/1000.0f), 
            cv::Point(tmp_mark_pt.x, tmp_mark_pt.y+15), cv::FONT_HERSHEY_COMPLEX, 0.8f, cv::Scalar(0,0,255), 1, CV_AA);
        tmp_mark_pt.y += 30;

        cv::putText(Canvas, 
            cv::format("ERROR[euclidean]-> [mm: %f]", eval_output.error_centroid), 
            cv::Point(tmp_mark_pt.x, tmp_mark_pt.y+15), cv::FONT_HERSHEY_COMPLEX, 0.8f, cv::Scalar(255,255,255), 1, CV_AA);
        //Predict
        cv::Point predict_gridpt = laser2grid(eval_output.output_predict.centroid_pt, 
                                            grid_global.base_pt[SRCFRAME], grid_global.mm2pixel);
        cv::circle(Canvas, predict_gridpt, 5, cv::Scalar(0,0,255), -1);
        //Ground-Truth
        cv::Point GroundTruth_gridpt = laser2grid(target_[TARGET_IDX].centroid_pt, 
                                                    grid_global.base_pt[SRCFRAME], grid_global.mm2pixel);
        cv::line(Canvas, GroundTruth_gridpt, predict_gridpt, cv::Scalar(255,255,255));
    }

    //origin axis
    cv::Point origin_pt = laser2grid(cv::Point2f(0.0f, 0.0f), grid_global.base_pt[SRCFRAME], grid_global.mm2pixel);
    cv::line(Canvas, cv::Point(0, origin_pt.y), 
                        cv::Point(grid_global.grid_col, origin_pt.y), cv::Scalar(0,255,0));
    cv::line(Canvas, cv::Point(origin_pt.x, 0), 
                        cv::Point(origin_pt.x, grid_global.grid_row), cv::Scalar(0,255,0));

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
    cv::Point2f sum_pt, mean_pt;

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
                cv::Point2f tmp_laser_m;
                tmp_laser_m.x = tmp_cloud.laser_coordinate_.x / 1000.0f;
                tmp_laser_m.y = tmp_cloud.laser_coordinate_.y / 1000.0f;

                _target.src_object_pts.push_back(tmp_cloud.laser_coordinate_);
                _target.src_object_pts_m.push_back(tmp_laser_m);
                sum_pt.x += tmp_cloud.laser_coordinate_.x;
                sum_pt.y += tmp_cloud.laser_coordinate_.y;
            }
        }
    }
    if(_target.src_object_pts.empty())
        ROS_ERROR("Contour of Robot is empty..");
    else
    {
        _target.src_dp_mat = tmp_canvas(tmp_rect);
        mean_pt.x = sum_pt.x / (float)_target.src_object_pts.size();
        mean_pt.y = sum_pt.y / (float)_target.src_object_pts.size();

        //get_ZeroMeaned(_target, mean_pt);
        //output_matching.x -= (mean_pt.x / 1000.0);
        //output_matching.y -= (mean_pt.y / 1000.0);
        output_robot.x = (mean_pt.x / 1000.0);       //m
        output_robot.y = (mean_pt.y / 1000.0);

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
void tracker::get_ZeroMeaned(allen::Target &_target, cv::Point2f _mean_pt)
{
    std::vector<cv::Point2f> tmp_mean_vec, tmp_mean_vec_m;

    for(int i = 0; i < (int)_target.src_object_pts.size(); i++)
    {
        cv::Point2f tmp_pt = _target.src_object_pts[i];
        tmp_pt.x -= _mean_pt.x;     //mm
        tmp_pt.y -= _mean_pt.y;
        tmp_mean_vec.push_back(tmp_pt);
        tmp_pt.x /= 1000.0f;        //mm to m
        tmp_pt.y /= 1000.0f;
        tmp_mean_vec_m.push_back(tmp_pt);
    }

    _target.src_object_pts.swap(tmp_mean_vec);
    _target.src_object_pts_m.swap(tmp_mean_vec_m);
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

        //set ROI for KCF
        roi_kcf = tmp_target.set_Region(grid_pt);
        if(!flag.get_flag(allen::FLAG::Name::ROIkcfOn) && !roi_kcf.empty())
        {
            tracker_KCF->init(gui.canvas, roi_kcf);
        }
        //update KCF
        if(!roi_kcf.empty())
        {
            bool valid_kcf = tracker_KCF->update(gui.canvas, roi_kcf);
            cv::rectangle(gui.canvas, roi_kcf, cv::Scalar(0,255,0), 2, 1);
        }

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

    //match_Robot_old(_target);
    match_Robot_new(_target);

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

        //tmp_laser.x *= 1000.0f;
        //tmp_laser.y *= 1000.0f;
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

    //Center
    cv::Point origin_pt = laser2grid(cv::Point2f(0.0f, 0.0f), grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);
    cv::Point test_pt = laser2grid(cv::Point2f(1000.0f, 0.0f), grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);
    cv::Point test1_pt = laser2grid(cv::Point2f(0.0f, 1000.0f), grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);
    cv::Point output_pt = laser2grid(cv::Point2f(output_robot.x*1000.0f, output_robot.y*1000.0f), 
                        grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);
    //origin axis
    cv::line(tmp_canvas, cv::Point(0, origin_pt.y), 
                        cv::Point(grid_tracker.grid_col, origin_pt.y), cv::Scalar(0,0,255));
    cv::line(tmp_canvas, cv::Point(origin_pt.x, 0), 
                        cv::Point(origin_pt.x, grid_tracker.grid_row), cv::Scalar(0,0,255));
    cv::circle(tmp_canvas, test_pt, 2, cv::Scalar(255,255,0), 5);
    cv::circle(tmp_canvas, test1_pt, 2, cv::Scalar(0,255,255), 5);
    //cv::circle(tmp_canvas, output_pt, 2, cv::Scalar(255,255,255), 5);


    cv::imshow(_win_name, tmp_canvas);
    //cv::waitKey(0);
}
std::vector<cv::Point2f> tracker::extract_Contour(allen::Target &_robot, std::vector<bag_t> &_bag_cloud, bool _flag)
{

    cv::Mat tmp_canvas = gui.canvas.clone();
    cv::Mat tmp_canvas_new(gui.canvas.rows, gui.canvas.cols, CV_8UC3, cv::Scalar(125,125,125));

    std::vector<cv::Point2f> output_contour;
    if(_flag)   //after matching
    {
        cv::Point2f centroid_robot(output_robot.x, output_robot.y);     //m
        centroid_robot.x *= 1000.0f;
        centroid_robot.y *= 1000.0f;

        for(int i = 0; i < (int)_bag_cloud.size(); i++)
        {
            bag_t tmp_cloud = _bag_cloud[i];
            for(int j = 0; j < (int)tmp_cloud.size(); j++)
            {
                allen::LaserPointCloud tmp_pt = tmp_cloud[j];           //mm
                float dist = get_dist2f(centroid_robot, tmp_pt.laser_coordinate_);
                //printf("dist: %f\n", dist);
                if(dist < 700.0f)
                {
                    cv::Point tmp_grid = 
                            laser2grid(tmp_pt.laser_coordinate_, grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);
                    cv::circle(tmp_canvas_new, tmp_grid, 3, cv::Scalar(0,0,255));
                    tmp_pt.laser_coordinate_.x /= 1000.0f;
                    tmp_pt.laser_coordinate_.y /= 1000.0f;
                    output_contour.push_back(tmp_pt.laser_coordinate_);
                }
            }
        }
        //cv::imshow("test", tmp_canvas_new);
        //cv::waitKey(0);
        return output_contour;
    }
    else        //init SRC
    {
        if((int)_robot.object_pts.size() == 0)  return std::vector<cv::Point2f>();
        //set search region
        cv::Rect tmp_region = _robot.set_Region(_robot.center_pt);
        cv::Mat tmp_contour = tmp_canvas(tmp_region);
        cv::Mat resize_contour;

        if(tmp_contour.cols <= 0 || tmp_contour.rows <= 0)
            return std::vector<cv::Point2f>();

        //cv::resize(tmp_contour, resize_contour, cv::Size(tmp_contour.cols*2.0, tmp_contour.rows*2.0), CV_INTER_LINEAR);

        //cv::imshow("dst", resize_contour);
        //cv::waitKey(0);
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
                    tmp_pt.laser_coordinate_.x /= 1000.0f;
                    tmp_pt.laser_coordinate_.y /= 1000.0f;
                    output_contour.push_back(tmp_pt.laser_coordinate_);
                }

            }
        }
        return output_contour;
    }
    
}
void tracker::match_Robot_old(std::vector<allen::Target> &_target)
{
    if((int)_target.size() != TARGETNUM)    return;

    std::vector<cv::Point2f> tmp_src_points = _target[ROBOT_IDX].src_object_pts;
    std::vector<cv::Point2f> tmp_dst_points; 

    ////////////////////////////////////////////////////////////////
    tmp_dst_points = extract_Contour(_target[ROBOT_IDX], bag_cloud_, flag.get_flag(allen::FLAG::Name::robotOn));
    ////////////////////////////////////////////////////////////////

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

        output_matching.translate(dst_frame, dst_frame_);       

        cv::flann::Index flann_idx(src_frame, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        allen::Frame tmp_output;
        double confidence = icp.run(dst_frame_, src_frame, tmp_output, flann_idx, draw);

        if(confidence)
        {
            //Initial-guess
            output_matching.x += tmp_output.x;
            output_matching.y += tmp_output.y;
            output_matching.th += tmp_output.th;
            output_matching.rearrange_Angle();

            output_robot = get_RobotPose(output_matching);
            if(!flag.get_flag(allen::FLAG::Name::robotOn))  flag.set_flag_on(allen::FLAG::Name::robotOn);

            //printf("output_matching: %lf, %lf, %lf\n", output_matching.x, output_matching.y, output_matching.th);
            //printf("output_robot: %lf, %lf, %lf\n\n", output_robot.x, output_robot.y, output_robot.th*radian2degree);
        }
        else
        {
            std::cout << "fail matching" << std::endl;
        }
        
    }
    else
    {
        ROS_ERROR("Not enough size of dst point-cloud");
        return;
    }
    

}
void tracker::match_Robot_new(std::vector<allen::Target> &_target)
{
    if((int)_target.size() != TARGETNUM)    return;

    std::vector<cv::Point2f> tmp_model_points = _target[ROBOT_IDX].src_object_pts_m;
    //std::vector<cv::Point2f> tmp_observ_points =    
                                //extract_Contour(_target[ROBOT_IDX], bag_cloud_, flag.get_flag(allen::FLAG::Name::robotOn));  //m
    std::vector<cv::Point2f> tmp_observ_points =    
                                extract_Contour(_target[ROBOT_IDX], bag_cloud_, 0);  //m

    bool valid_run = true;
    if((int)tmp_model_points.size() <= 0)     valid_run = false;
    if((int)tmp_observ_points.size() <= 0)    valid_run = false;
    if(valid_run)
    {
        cv::Mat draw, model_frame, observ_frame;
        draw = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(255,255,255));
        if(flag.get_flag(allen::FLAG::Name::robotOn))
            model_frame = model_frame_output.clone();        //from
        else
            model_frame = cv::Mat((int)tmp_model_points.size(), 2, CV_32FC1, tmp_model_points.data());
        observ_frame = cv::Mat((int)tmp_observ_points.size(), 2, CV_32FC1, tmp_observ_points.data());     //to

        cv::flann::Index flann_idx(observ_frame, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        allen::Frame output;
        double success = icp.run(model_frame, observ_frame, output, flann_idx, draw);

        if(success)
        {
            if(!flag.get_flag(allen::FLAG::Name::robotOn))  flag.set_flag_on(allen::FLAG::Name::robotOn);
            model_frame_output = model_frame.clone();

            //get robot pose
            robot_RT = output_robot.add_Motion(output);      //add RT from ICP output
            if(robot_RT.empty())    return;

            output_robot.x = (double)robot_RT.at<float>(0, 2);
            output_robot.y = (double)robot_RT.at<float>(1, 2);
            output_robot.th += output.th;       //degree
            output_robot.rearrange_Angle();
            //std::cout << "output_robot_th: " << output_robot.th << std::endl;
            //output_robot = get_RobotPose(output_matching);
        }

        cv::imshow("matching", draw);
        //printf("confidence: %lf\n", success);

    }
    


}
allen::Frame tracker::get_RobotPose(allen::Frame _icp_pose)
{
    allen::Frame tmp_frame;
    //tmp_frame.th = _icp_pose.th * degree2radian * -1.0;    //radian
    tmp_frame.th = _icp_pose.th * degree2radian;    //radian
    tmp_frame.x = _icp_pose.x * cos(tmp_frame.th) - _icp_pose.y * sin(tmp_frame.th);
    tmp_frame.y = _icp_pose.x * sin(tmp_frame.th) + _icp_pose.y * cos(tmp_frame.th);
    //tmp_frame.x *= -1.0;   //m
    //tmp_frame.y *= -1.0;   //m

    return tmp_frame;
}
std::vector<cv::Point2f> tracker::get_dstContour(allen::Frame _robot_pose, cv::Mat &_dst_inlier)
{
    if(_dst_inlier.rows < 10)
    {
        return std::vector<cv::Point2f>();
    }

    cv::Mat tmp_canvas(gui.canvas.rows, gui.canvas.cols, CV_8UC3, cv::Scalar(0,0,0));
    std::vector<cv::Point2f> output_vec;

    for(int i = 0; i < _dst_inlier.rows; i++)
    {
        cv::Point2f point;
        point.x = _dst_inlier.at<float>(i, 0) * 1000.0f;
        point.y = _dst_inlier.at<float>(i, 1) * 1000.0f;
        //printf("dst_inlier-> [cnt: %d][x: %f, y: %f]\n", i, point.x, point.y);
        output_vec.push_back(point);

        //debug
        cv::Point img_point;
        img_point = laser2grid(point, grid_tracker.base_pt[SRCFRAME], grid_tracker.mm2pixel);
        //printf("dst_inlier-> [cnt: %d][gx: %d, gy: %d]\n\n", i, img_point.x, img_point.y);
        cv::circle(tmp_canvas, img_point, 2, cv::Scalar(255, 255, 255), -1);
    }
    cv::imshow("inlier", tmp_canvas);
    //cv::waitKey(0);
    //std::cout << std::endl;
    return output_vec;
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
    if(flag.get_flag(allen::FLAG::Name::calibration))
    {
        calibrator_server::Calibrate_laser srv;
        srv.request.calibrate = true;
        if(client_calib.call(srv))
        {
            ROS_INFO("Response[calibration]: %d", srv.response.success);
            gui.clicked_button(_canvas, gui.b_calib);
        }
        else
        {
            ROS_ERROR("Fail to call service [calibration]");
            gui.clicked_button(_canvas, gui.b_calib);
        }
        flag.set_flag_off(allen::FLAG::Name::calibration);
    }
    if(flag.get_flag(allen::FLAG::Name::setRect) && m_up)
        set_Target(target_, gui.drag_rect);
    if(flag.get_flag(allen::FLAG::Name::reset))
    {
        reset_mtx.lock();
        //logging
        assembly_evalData();
        log_error.write("bag", eval_mat);
        log_error.end();

        //reset
        flag.resetFlag();
        target_.clear();
        gui.reset(_canvas);
        output_matching = allen::Frame();
        output_robot = allen::Frame();
        icp.reset();
        predict_posi_ = allen::Frame();
        eval_output.reset();
        eval_bag.clear();
        eval_mat = cv::Mat();
        model_frame_output = cv::Mat();
        robot_RT = cv::Mat();
        roi_kcf = cv::Rect2d();

        reset_mtx.unlock();
    }
}
void tracker::get_PredictCoordinate(allen::Frame _predict_posi, cv::Mat _robot_RT)
{
    if(_robot_RT.empty())   return;

    cv::Mat person_RT, new_person_RT;
    person_RT = cv::Mat(3, 1, CV_32FC1, cv::Scalar(1));
    //person_RT = cv::Mat::eye(3, 3, CV_32FC1);

    //float p_r = (float)_predict_posi.th / 180.0f * (float)CV_PI;
    //std::cout << "predict_th: " << _predict_posi.th << std::endl;
    //printf("predict_th: %lf\n", _predict_posi.th);
    //person_RT.at<float>(0,0) = cos(p_r);
    //person_RT.at<float>(0,1) = -sin(p_r);
    //person_RT.at<float>(1,0) = sin(p_r);
    //person_RT.at<float>(1,1) = cos(p_r);

    person_RT.at<float>(0,0) = (float)_predict_posi.x / 1000.0f;    //x (mm2m)
    person_RT.at<float>(1,0) = (float)_predict_posi.y / 1000.0f;    //y

    //new_person_RT = person_RT * _robot_RT;      //t_pose = d_pose * t-1_pose
    new_person_RT = _robot_RT * person_RT;      //t_pose = d_pose * t-1_pose

    //std::cout << "calced new person RT: " << std::endl << new_person_RT << std::endl;
    //assign new person point
    eval_output.output_predict.centroid_pt.x = (double)new_person_RT.at<float>(0,0) * 1000.0;   //x (m2mm)
    eval_output.output_predict.centroid_pt.y = (double)new_person_RT.at<float>(1,0) * 1000.0;   //y (m2mm)

    cv::Point2f target_centroid = target_[TARGET_IDX].centroid_pt;      //ground-truth
    if(std::isinf(target_centroid.x) || std::isinf(target_centroid.y))      return;

    eval_output.error_centroid = get_dist2f(target_centroid, eval_output.output_predict.centroid_pt);
    //printf("error_centroid: %f\n\n", eval_output.error_centroid);

    //feed
    allen::Evaluation tmp_eval;
    tmp_eval.time = ros::Time::now();
    tmp_eval.error_target2GT = eval_output.error_centroid;
    tmp_eval.GT_world = target_centroid;
    tmp_eval.predict_world = eval_output.output_predict.centroid_pt;

    for(int i = 0; i < TARGETNUM; i++)
    {
        if(i < 2)   continue;
        cv::Point2f tmp_pt = target_[i].centroid_pt;
        tmp_eval.non_target_world.push_back(tmp_pt);
    }

    eval_bag.push_back(tmp_eval);

}
void tracker::assembly_evalData(void)
{
    if((int)eval_bag.size() == 0)
    {
        ROS_ERROR("Eval_bag is empty..");
        return;
    }

    int default_size = 6;
    int col_size = default_size + (TARGETNUM - 2) * 2;
    cv::Mat logging_mat((int)eval_bag.size(), col_size, CV_64FC1, cv::Scalar(0.0f));

    for(int i = 0; i < (int)eval_bag.size(); i++) 
    {
        logging_mat.at<double>(i, 0) = eval_bag[i].time.toSec();
        logging_mat.at<double>(i, 1) = (double)eval_bag[i].error_target2GT;
        logging_mat.at<double>(i, 2) = (double)eval_bag[i].predict_world.x;
        logging_mat.at<double>(i, 3) = (double)eval_bag[i].predict_world.y;
        logging_mat.at<double>(i, 4) = (double)eval_bag[i].GT_world.x;
        logging_mat.at<double>(i, 5) = (double)eval_bag[i].GT_world.y;

        int col_idx = default_size;
        for(int j = 0; j < (int)eval_bag[i].non_target_world.size(); j++)
        {
            cv::Point2f tmp_pt = eval_bag[i].non_target_world[j];
            logging_mat.at<double>(i, col_idx) = (double)tmp_pt.x;
            logging_mat.at<double>(i, col_idx+1) = (double)tmp_pt.y;
            col_idx += 2;
        }
    }

    eval_mat = logging_mat.clone();

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

        predict_mtx.lock();
        eval_output.predict_posi = predict_posi_;
        predict_mtx.unlock();
    }

    if(!flag.get_flag(allen::FLAG::Name::dataOn))   flag.set_flag_on(allen::FLAG::Name::dataOn);
}
void tracker::runLoop(void)
{
    ros::Rate r(20);
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
                if(flag.get_flag(allen::FLAG::Name::predictOn))
                {
                    get_PredictCoordinate(eval_output.predict_posi, robot_RT);
                }
            }
        }
        ros::spinOnce();
        r.sleep();
    }

}

