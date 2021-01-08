#include "calibrator/matcher.hpp"

void matcher::initSubscriber()
{
    scan_1_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_1", 10, boost::bind(&matcher::scan_callback, this, _1, 0));
    scan_2_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_2", 10, boost::bind(&matcher::scan_callback, this, _1, 1));
    scan_3_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_3", 10, boost::bind(&matcher::scan_callback, this, _1, 2));
    scan_4_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_4", 10, boost::bind(&matcher::scan_callback, this, _1, 3));
}
void matcher::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg, int idx)
{
    if(!sensors[idx]->get_tf_flag)
        sensors[idx]->get_tf_flag = sensors[idx]->get_tf();

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
void matcher::transform(cv::Mat& from, cv::Mat& to, allen::Frame frame)
{
	if(from.type() == 6)	//64FC1
	{
		to = cv::Mat(from.size(), CV_64FC1);
		double r = frame.th * degree2radian;
		for(int i=0; i<from.rows; i++)
		{
			double x = from.at<double>(i, 0);	
			double y = from.at<double>(i, 1);	
		
			to.at<double>(i, 0) = x*cos(r) - y*sin(r) + frame.x;
			to.at<double>(i, 1) = x*sin(r) + y*cos(r) + frame.y;
		}
	}
	else if(from.type() == 5)	//32FC1
	{
		to = cv::Mat(from.size(), CV_32FC1);
		float r = (float)(frame.th * degree2radian);
		for(int i=0; i<from.rows; i++)
		{
			float x = from.at<float>(i, 0);	
			float y = from.at<float>(i, 1);	
		
			to.at<float>(i, 0) = x*cos(r) - y*sin(r) + (float)frame.x;
			to.at<float>(i, 1) = x*sin(r) + y*cos(r) + (float)frame.y;
		}
	}
}
double matcher::getPointDist(cv::Mat& data, int idx1, int idx2)
{
	if(data.type() == 6)
	{
		double x1 = data.at<double>(idx1, 0);
		double y1 = data.at<double>(idx1, 1);
		
		double x2 = data.at<double>(idx2, 0);
		double y2 = data.at<double>(idx2, 1);

		double dx = x2 - x1;
		double dy = y2 - y1;
//		double dist = std::sqrt(dx*dx + dy*dy);
		double dist = dx*dx + dy*dy;
		return dist;
	}
	else if(data.type() == 5)
	{
		float x1 = data.at<float>(idx1, 0);
		float y1 = data.at<float>(idx1, 1);
		
		float x2 = data.at<float>(idx2, 0);
		float y2 = data.at<float>(idx2, 1);

		float dx = x2 - x1;
		float dy = y2 - y1;
//		float dist = std::sqrt(dx*dx + dy*dy);
		float dist = dx*dx + dy*dy;
		return (double)dist;
	}
	else
	{
		ROS_ERROR("[matcher]-> Unknown type for getPointDist");
		return 0.0;	
	}
}
void matcher::centroid(cv::Mat& target, cv::Mat& center)
{
	if(target.type() == 6)
	{
		double sum_x = 0.0;
		double sum_y = 0.0;
		for(int i=0; i<target.rows; i++)
		{
			sum_x += target.at<double>(i, 0);
			sum_y += target.at<double>(i, 1);
		}

		double avr_x = sum_x / (double)(target.rows -1);
		double avr_y = sum_y / (double)(target.rows -1);
		for(int i=0; i<target.rows; i++)
		{
			target.at<double>(i, 0) -= avr_x;
			target.at<double>(i, 1) -= avr_y;
		}

		center = cv::Mat(2, 1, CV_64FC1, cv::Scalar(0));
		center.at<double>(0, 0) = avr_x;
		center.at<double>(1, 0) = avr_y;
	}
	else if(target.type() == 5)
	{
		double sum_x = 0.0;
		double sum_y = 0.0;
		for(int i=0; i<target.rows; i++)
		{
			sum_x += (double)target.at<float>(i, 0);
			sum_y += (double)target.at<float>(i, 1);
		}


		double avr_x = sum_x / (double)target.rows;
		double avr_y = sum_y / (double)target.rows;

		for(int i=0; i<target.rows; i++)
		{
			target.at<float>(i, 0) -= (double)avr_x;
			target.at<float>(i, 1) -= (double)avr_y;
		}


		center = cv::Mat(2, 1, CV_32FC1, cv::Scalar(0));
		center.at<float>(0, 0) = (double)avr_x;
		center.at<float>(1, 0) = (double)avr_y;
	}
	else
		ROS_ERROR("[LaserMatcher]-> Known type for centroid");
}
void matcher::match(cv::flann::Index& flann_idx, cv::Mat& from, cv::Mat& to, 
			cv::Mat& from_inlier, cv::Mat& to_inlier, double ratio, cv::Mat& draw)
{
	bool draw_process = !draw.empty();

	cv::Mat index, dists;
	//from.rows == index.rows == dists.rows
	flann_idx.knnSearch(from, index, dists, 2, cv::flann::SearchParams());	

	cv::Mat index_counter(to.rows, 1, CV_32SC1, cv::Scalar(0));
	for(int i=0; i<from.rows; i++)
	{
		int idx = index.at<int>(i, 0);
		if(idx < 0 || idx >= index_counter.rows) 
			continue;
		index_counter.at<int>(idx, 0) = index_counter.at<int>(idx, 0) + 1;
	}

	std::vector<float> temp_in;
	temp_in.reserve(from.rows);
	for(int i=0; i<from.rows; i++)
	{
		int idx = index.at<int>(i, 0);
		int idx_second = index.at<int>(i, 1);

		bool in = true;

		if(index_counter.at<int>(idx, 0) >= 5)
			in = false;
		else if(getPointDist(to, idx, idx_second) > 1.0f)
			in = false;

		if(!in) index.at<int>(i, 0) = -1;
		else temp_in.push_back(dists.at<float>(i, 0));
	}

	if(temp_in.size() <= 4) return;

	cv::Mat in_dist(temp_in.size(), 1, CV_32FC1);	
	memcpy(in_dist.data, temp_in.data(), sizeof(float)*temp_in.size());
	cv::Mat in_dist_sorted;
	cv::sort(in_dist, in_dist_sorted, CV_SORT_EVERY_COLUMN || CV_SORT_ASCENDING);

	int median_pos = (int)((float)in_dist_sorted.rows*0.6f);
	float threshold = 100.0f*in_dist_sorted.at<float>(median_pos, 0);

	int inlier_counter = 0;
	for(int i=0; i<from.rows; i++)
	{
		bool in = true;

		int idx = index.at<int>(i, 0);

		if(idx < 0 || idx >= to.rows) 
		{	
			index.at<int>(i, 0) = -1;
			continue;
		}

		if(ratio < 0)
			in = true;
		else if(dists.at<float>(i, 0) > threshold)
			in = false;

		if(in) inlier_counter++;
		else index.at<int>(i, 0) = -1;
	}

	from_inlier = cv::Mat(inlier_counter, 2, from.type());
	to_inlier = cv::Mat(inlier_counter, 2, to.type());

	int counter = 0;	
	for(int i=0; i<index.rows; i++)
	{
		int idx = index.at<int>(i, 0);
		if(idx == -1)
			continue;

		from.row(i).copyTo(from_inlier.row(counter));
		to.row(idx).copyTo(to_inlier.row(counter));
		counter++;
	}

	if(draw_process)
	{
		float min_col = to.at<float>(0, 1);
		float max_col = to.at<float>(0, 1);
		float min_row = to.at<float>(0, 0);
		float max_row = to.at<float>(0, 0);
		for(int i=1; i<to.rows; i++)
		{
			if(min_col > to.at<float>(i, 1)) min_col = to.at<float>(i, 1);
			if(max_col < to.at<float>(i, 1)) max_col = to.at<float>(i, 1);
			if(min_row > to.at<float>(i, 0)) min_row = to.at<float>(i, 0);
			if(max_row < to.at<float>(i, 0)) max_row = to.at<float>(i, 0);
		}

		if(max_col-min_col != 0 && max_row-min_row != 0)
		{
			cv::Mat plot1(500, draw.cols ,CV_8UC3, cv::Scalar(255, 255, 255));			

			float m2p1 = std::min(
				((float)plot1.cols - 20.0f)/(max_col-min_col), 
				((float)plot1.rows - 20.0f)/(max_row-min_row)
			);

			float c_col = (max_col+min_col) / 2.0f;
			float c_row = (max_row+min_row) / 2.0f;


			for(int i=0; i<to.rows; i++)
			{
				cv::Point2f p(
					plot1.cols/2 - m2p1*(to.at<float>(i, 1) - c_col),
					plot1.rows/2 - m2p1*(to.at<float>(i, 0) - c_row)
				);	
				cv::circle(plot1, p, 1, cv::Scalar(0, 0, 255));			//to
			}

			for(int i=0; i<from.rows; i++)
			{
				cv::Point2f p(
					plot1.cols/2 - m2p1*(from.at<float>(i, 1) - c_col),
					plot1.rows/2 - m2p1*(from.at<float>(i, 0) - c_row)
				);	
				cv::circle(plot1, p, 1, cv::Scalar(255, 0, 0));			//from
			}

			for(int i=0; i<from_inlier.rows; i++)
			{
				cv::Point2f from_p(
					plot1.cols/2 - m2p1*(from_inlier.at<float>(i, 1) - c_col),
					plot1.rows/2 - m2p1*(from_inlier.at<float>(i, 0) - c_row)
				);	
				cv::Point2f to_p(
					plot1.cols/2 - m2p1*(to_inlier.at<float>(i, 1) - c_col),
					plot1.rows/2 - m2p1*(to_inlier.at<float>(i, 0) - c_row)
				);
				cv::line(plot1, from_p, to_p, cv::Scalar(0, 255, 0));	//inlier
			}

			draw.push_back(plot1);

			float height = 100;
			cv::Mat dist_draw(height, draw.cols, CV_8UC3, cv::Scalar(255, 255, 255));	
			float norm = std::sqrt(in_dist_sorted.at<float>(in_dist_sorted.rows-1, 0));

			for(int i=0; i<in_dist_sorted.rows; i++)
			{
				float val = std::sqrt(in_dist_sorted.at<float>(i, 0))/norm;
				cv::Point2f p1(i, height);
				cv::Point2f p2(i, height-height*val);
				cv::line(dist_draw, p1, p2, cv::Scalar(0, 0, 0));
			}

			float threshold_line = std::sqrt(threshold) / norm;
			float h = height-height*threshold_line;
			cv::line(dist_draw, 
				cv::Point2f(0, h), cv::Point2f(dist_draw.cols, h), 
				cv::Scalar(0, 0, 255));

			draw.push_back(dist_draw);
		}
	}
}
bool matcher::run(cv::Mat &from, cv::Mat &to, allen::Frame &output, cv::flann::Index &flann_idx, cv::Mat &draw)
{
    bool draw_result = !draw.empty();
	output.x = 0.0;
	output.y = 0.0;
	output.th = 0.0;

	if(from.cols != 2 || to.cols != 2)
	{
		ROS_ERROR("[matcher]-> cols of from or to mat is not 2");
		return false;
	}

	if(from.rows <= 10 || to.rows <= 10)
	{
		ROS_ERROR("[matcher]-> Not enough data. from: %d, to: %d", from.rows, to.rows);
		return false;
	}

	double ratio = 1.0;	
	cv::Mat points;
	
	//cona::Timer iter_timer;
	//iter_timer.tic();
	int iter=0;

	cv::Mat matching_draw;

	double max_th = 20.0 / 180.0 * (double)M_PI;

	for(; iter<50; iter++)
	{
		matching_draw = cv::Mat(1, 500, CV_8UC3, cv::Scalar(255, 255, 255)); 
		transform(from, points, output);
		cv::Mat from_inlier, to_inlier;
		match(flann_idx, points, to, from_inlier, to_inlier, ratio, matching_draw);

		if(!matching_draw.empty())
		{
			cv::putText(matching_draw, 
				cv::format("iter: %d x: %.5lf y: %.5lf th: %.5lf", 
					iter, output.x, output.y, output.th), 
				cv::Point(15, 25), 0, 0.5, cv::Scalar(0, 0, 0)
			);
			bool success = cv::imwrite(cv::format("/home/allenkim/log/eval/%lf.jpg", 
				ros::Time::now().toSec()), matching_draw);
		}

		if(from_inlier.rows <= 10 || to_inlier.rows <= 10)
		{
			break;
		}

		cv::Mat from_center;
		centroid(from_inlier, from_center);

		cv::Mat to_center;
		centroid(to_inlier, to_center);

		cv::Mat M = from_inlier.t() * to_inlier;
		cv::SVD svd(M);

		cv::Mat temp_R = svd.vt.t() * svd.u.t();
		cv::Mat temp_T = to_center - temp_R * from_center;

		float dx = temp_T.at<float>(0, 0);
		float dy = temp_T.at<float>(1, 0);
		float dth = acos(std::min(temp_R.at<float>(0, 0), 1.0f));
		if(temp_R.at<float>(0, 1) > 0) dth = -dth;

		if(dth > max_th) dth = max_th;
		else if(dth < -max_th) dth = -max_th;

		output.x += dx;
		output.y += dy;
		output.th += dth / degree2radian;

//		if(fabs(dx)<0.01f && fabs(dy)<0.01f && fabs(dth) < 0.001f)
		if(fabs(dx)<0.005f && fabs(dy)<0.005f && fabs(dth) < 0.0005f)
		{
			transform(from, points, output);
			break;
		}
	}

	if(1)
	{
		points.copyTo(from);

	}

	if(draw_result)
	{
		float min_col = to.at<float>(0, 1);
		float max_col = to.at<float>(0, 1);
		float min_row = to.at<float>(0, 0);
		float max_row = to.at<float>(0, 0);
		for(int i=1; i<to.rows; i++)
		{
			if(min_col > to.at<float>(i, 1)) min_col = to.at<float>(i, 1);
			if(max_col < to.at<float>(i, 1)) max_col = to.at<float>(i, 1);
			if(min_row > to.at<float>(i, 0)) min_row = to.at<float>(i, 0);
			if(max_row < to.at<float>(i, 0)) max_row = to.at<float>(i, 0);
		}

		if(max_col-min_col != 0 && max_row-min_row != 0)
		{
			float m2p = std::min(
				((float)draw.cols - 20.0f)/(max_col-min_col), 
				((float)draw.rows - 20.0f)/(max_row-min_row)
			);

			float c_col = (max_col+min_col) / 2.0f;
			float c_row = (max_row+min_row) / 2.0f;

			for(int i=0; i<to.rows; i++)
			{
				cv::Point2f p(
					draw.cols/2 - m2p*(to.at<float>(i, 1) - c_col), draw.rows/2 - m2p*(to.at<float>(i, 0) - c_row)
				);	
				cv::circle(draw, p, 3, cv::Scalar(0, 0, 255));		//to(t)
			}

			cv::Mat init_points = from.clone();
//			transform(from, init_points, init);
			for(int i=0; i<points.rows; i++)
			{
				cv::Point2f p(
					draw.cols/2 - m2p*(init_points.at<float>(i, 1) - c_col),
					draw.rows/2 - m2p*(init_points.at<float>(i, 0) - c_row)
				);	
				//cv::circle(draw, p, 1, cv::Scalar(255, 0, 0));		//from(t-n)
			}

			cv::Mat output_points;
			output.translate(init_points, output_points);

			cv::Mat result_points = output_points;
			for(int i=0; i<result_points.rows; i++)
			{
				cv::Point2f p(
					draw.cols/2 - m2p*(result_points.at<float>(i, 1) - c_col),	
					draw.rows/2 - m2p*(result_points.at<float>(i, 0) - c_row)
				);	
				cv::circle(draw, p, 1, cv::Scalar(0, 255, 0));		//result
			}

			//cv::putText(draw, cv::format("iter: %5.1lf i: %d", 
					//iter_timer.get_ms(), iter),
				//cv::Point(15, 25), 0, 0.8, cv::Scalar(0, 0, 0));
		}
	}


    return true;
}
void matcher::getTransformation(void)
{
    cv::Mat src_frame, dst_frame, draw;
    int src_frame_size = (int)sensors[SRCFRAME]->pointcloud.size();
    std::vector<allen::Frame> tmp_Frames;
    for(int i = 0; i < SENSORNUM; i++)
        tmp_Frames.push_back(allen::Frame(999.0, 999.0, 999.0));
    int count = 0;
    for(int i = 0; i < SENSORNUM; i++)      //other sensors -> dst
    {
        if(i == SRCFRAME)   continue;
        int dst_frame_size = (int)sensors[i]->pointcloud.size();
        std::vector<cv::Point2f> tmp_src = sensors[SRCFRAME]->cvtFloat(sensors[SRCFRAME]->pointcloud);
        std::vector<cv::Point2f> tmp_dst = sensors[i]->cvtFloat(sensors[i]->pointcloud);
        //std::vector<cv::Point2f> tmp_dst = sensors[0]->cvtFloat(sensors[0]->pointcloud);

        draw = cv::Mat(grid.grid_row, grid.grid_col, CV_8UC3, cv::Scalar(255,255,255));
        src_frame = cv::Mat(src_frame_size, 2, CV_32FC1, tmp_src.data());     //from
        dst_frame = cv::Mat(dst_frame_size, 2, CV_32FC1, tmp_dst.data());        //to
        cv::flann::Index flann_idx(dst_frame, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
        //cv::imshow("draw", draw);

        allen::Frame tmp_output;
        bool success = run(src_frame, dst_frame, tmp_output, flann_idx, draw);
        printf("output[%d]-> x: %lf, y: %lf, th: %f\n", i, tmp_output.x, tmp_output.y, tmp_output.th);

        if(success)
        {
            tmp_Frames[i] = tmp_output;
            count++;
        }

        if(src_frame.rows < 2 || dst_frame.rows < 2)    
        {
            flag_calibOn = false;
            return;
        }
    }
    std::cout << std::endl;

    output_frames.swap(tmp_Frames);
    if(count == SENSORNUM-1)    flag_calibOn = true;
    else                        flag_calibOn = false;
}
void matcher::calibrate_Frames(std::vector<allen::Frame> &_output_frames)
{
    if((int)_output_frames.size() != SENSORNUM || !flag_calibOn)
    {
        ROS_ERROR("[matcher]Not enough transformation(%d/%d)", (int)_output_frames.size(), SENSORNUM-1);
        return;
    }

    for(int i = 0; i < (int)_output_frames.size(); i++)
    {
        allen::Frame tmp_tf = _output_frames[i];
        tmp_tf.x *= -1;

        printf("[%d]-> %lf, %lf, %lf\n", i, tmp_tf.x, tmp_tf.y, tmp_tf.th);

        int size = (int)sensors[i]->pointcloud.size();
        bag_t tmp_calibrated_cloud;
        tmp_calibrated_cloud.reserve(size);

        for(int j = 0; j < size; j++)
        {
            float x = sensors[i]->pointcloud[j].laser_coordinate_.x;
            float y = sensors[i]->pointcloud[j].laser_coordinate_.y;
            float n_x, x_y;
            printf("[before]-> x: %f, y: %f\n", x, y);
            
            allen::LaserPointCloud tmp_lpc;
            tmp_lpc.laser_stamp_ = sensors[i]->pointcloud[j].laser_stamp_;
            tmp_lpc.angle = sensors[i]->pointcloud[j].angle;

            if(i == SRCFRAME)
                tmp_lpc.laser_coordinate_ = cv::Point2f(x, y);
            else
            {
                n_x = x * cos(tmp_tf.th) -  y * sin(tmp_tf.th) + tmp_tf.x;
                n_y = x * sin(tmp_tf.th) + y * cos(tmp_tf.th) + tmp_tf.y;
                tmp_lpc.laser_coordinate_ = cv::Point2f(n_x, n_y);
            }

            printf("[after]-> x: %f, y: %f\n", n_x, n_y);
            tmp_calibrated_cloud.push_back(tmp_lpc);

            //tf::Vector3 p(x, y, 1);
            //tf::Matrix3x3 tmp_R(0);
            //tmp_R.at<float>(0, 0) = cos(tmp_tf.th);
            //tmp_R.at<float>(0, 1) = -sin(tmp_tf.th);
            //tmp_R.at<float>(1, 0) = sin(tmp_tf.th);
            //tmp_R.at<float>(1, 1) = cos(tmp_tf.th);
            //tmp_R.at<float>(0, 2) = tmp_tf.x;
            //tmp_R.at<float>(1, 2) = tmp_tf.y;
            //tmp_R.at<float>(2, 2) = 1.0;
            //tf::vector3 new_p = tmp_R * p;

        }
        std::cout << std::endl;
        sensors[i]->calibrated_pointcloud.swap(tmp_calibrated_cloud);
    }

}
void matcher::display_Globalmap(void)
{
    if(!imshow)                     return;
    if((int)sensors.size() == 0)    return;

    allen::Grid_param grid_global;
    cv::Mat Canvas(grid_global.grid_row, grid_global.grid_col, CV_8UC3, cv::Scalar(0,0,0));

    float margin_grid = 300.0f;

    grid_global.base_pt.push_back(cv::Point2f(margin_grid, margin_grid));
    grid_global.base_pt.push_back(cv::Point2f(margin_grid, (float)grid_global.grid_row-margin_grid));
    grid_global.base_pt.push_back(cv::Point2f((float)grid_global.grid_col-margin_grid, margin_grid));
    grid_global.base_pt.push_back(cv::Point2f((float)grid_global.grid_col-margin_grid, (float)grid_global.grid_row-margin_grid));

    cv::Point2f tmp_base_pt = grid_global.base_pt[SRCFRAME];

    for(int i = 0; i < (int)sensors.size(); i++)
    {
        bag_t tmp_pointcloud = sensors[i]->pointcloud;
        cv::Scalar tmp_scalar;
        if(i == SRCFRAME)
            tmp_scalar = cv::Scalar(0,255,0);   //green
        else
            tmp_scalar = sensors[i]->pointcolor;
        //cv::Point2f tmp_base_pt = grid_global.base_pt[i];
        //if(i == 0 || i == 2)    continue;

        for(int j = 0; j < (int)tmp_pointcloud.size(); j++)
        {
            allen::LaserPointCloud tmp_lpc = tmp_pointcloud[j];
            cv::Point2f tmp_pt_mm = tmp_lpc.laser_coordinate_ * 1000.0f;     //m to mm
            if(std::isinf(tmp_pt_mm.x) || std::isinf(tmp_pt_mm.y))  continue;
            cv::Point2f tmp_pt_grid;
            tmp_pt_grid.x = tmp_base_pt.x - tmp_pt_mm.y * grid_global.mm2pixel;
            tmp_pt_grid.y = tmp_base_pt.y - tmp_pt_mm.x * grid_global.mm2pixel;
            //tmp_pt_grid.x = grid_global.robot_col - tmp_pt_mm.y * grid_global.mm2pixel;
            //tmp_pt_grid.y = grid_global.robot_row - tmp_pt_mm.x * grid_global.mm2pixel;

            cv::circle(Canvas, tmp_pt_grid, 5, tmp_scalar, -1);
        }
    }
    cv::line(Canvas, cv::Point(0, tmp_base_pt.y), 
                        cv::Point(grid_global.grid_col, tmp_base_pt.y), cv::Scalar(0,0,255));
    cv::line(Canvas, cv::Point(tmp_base_pt.x, 0), 
                        cv::Point(tmp_base_pt.x, grid_global.grid_row), cv::Scalar(0,0,255));
    //cv::imshow("Canvas", Canvas);
    Globalmap = Canvas.clone();
}
void matcher::get_syncData()
{
    flag_dataOn = false;
    for(int i = 0; i < (int)sensors.size(); i++)
        if(sensors[i]->pointcloud.size() == 0)  return;

    std::vector<bag_t*> tmp_bag_cloud;
    for(int i = 0; i < (int)sensors.size(); i++)
    {
        sensors[i]->mtx_scan.lock();
        tmp_bag_cloud.push_back(new bag_t(sensors[i]->pointcloud));
        //printf("in for loop: size[%d]-> %d\n", i, (int)tmp_bag_cloud[i]->size());
        sensors[i]->mtx_scan.unlock();
    }
    bag_cloud_.swap(tmp_bag_cloud);
    if(!flag_dataOn)    flag_dataOn = true;

    //release
    std::vector<bag_t*>::iterator it;
    for(it=tmp_bag_cloud.begin(); it!=tmp_bag_cloud.end(); it++)
        delete *it;
}
void matcher::runLoop()
{
    ros::Rate r(15);
    while (ros::ok())
    {
        get_syncData();
        if(flag_dataOn)
        {
            getTransformation();
            calibrate_Frames(output_frames);
            display_Globalmap();
        }

        //display
        for(int i = 0; i < (int)sensors.size(); i++)
            cv::imshow(sensors[i]->child_frame, sensors[i]->Grid_local);
        cv::imshow("Globalmap", Globalmap);
        cv::waitKey(10);

        ros::spinOnce();
        r.sleep();
    }
    
}
