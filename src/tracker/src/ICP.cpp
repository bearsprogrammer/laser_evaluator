#include "tracker/ICP.hpp"

void ICP::transform(cv::Mat& from, cv::Mat& to, allen::Frame frame)
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
double ICP::getPointDist(cv::Mat& data, int idx1, int idx2)
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
        std::cout << "[ICP]-> Unknown type for getPointDist" << std::endl;
        return 0.0;	
    }
}
void ICP::centroid(cv::Mat& target, cv::Mat& center)
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
        std::cout << "[ICP]-> Unknown type for centroid" << std::endl;
}
void ICP::match(cv::flann::Index& flann_idx, cv::Mat& from, cv::Mat& to
	, cv::Mat& from_inlier, cv::Mat& to_inlier, double& inlier_midian_dist
	, cv::Mat& draw)
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
    inlier_midian_dist = (double)in_dist_sorted.at<float>(median_pos, 0);

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

        if(dists.at<float>(i, 0) > threshold)
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
                cv::circle(plot1, p, 1, cv::Scalar(0, 0, 255));
            }

            for(int i=0; i<from.rows; i++)
            {
                cv::Point2f p(
                    plot1.cols/2 - m2p1*(from.at<float>(i, 1) - c_col),
                    plot1.rows/2 - m2p1*(from.at<float>(i, 0) - c_row)
                );	
                cv::circle(plot1, p, 1, cv::Scalar(255, 0, 0));
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
                cv::line(plot1, from_p, to_p, cv::Scalar(0, 255, 0));	
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
bool ICP::run(cv::Mat &from, cv::Mat &to, allen::Frame &output, cv::flann::Index &flann_idx, cv::Mat &draw)
{
	double output_confidence = 0.0;
	double threashold_inlier_midian_dist = 0.008;
	double threashold_low_ratio = 0.1;

	bool draw_result = !draw.empty();

	output.x = 0.0;
	output.y = 0.0;
	output.th = 0.0;

	if(from.cols != 2 || to.cols != 2)
	{
		ROS_ERROR("cols of from or to mat is not 2");
		return output_confidence;
	}

	if(from.rows <= 10 || to.rows <= 10)
	{
		ROS_WARN("Not enough data. from: %d, to: %d", 
			from.rows, to.rows);
		return output_confidence;
	}

	cv::Mat points;
	
	int iter=0;

	cv::Mat matching_draw;

	double max_th = 20.0 / 180.0 * (double)M_PI;

	double inlier_rate = 0.0;
	double inlier_midian_dist = 0.0;	
	for(; iter<50; iter++)
	{
		if(draw_result)
			matching_draw = cv::Mat(1, 500, CV_8UC3, cv::Scalar(255, 255, 255));

		transform(from, points, output);

		cv::Mat from_inlier, to_inlier;
		match(flann_idx, points, to, from_inlier, to_inlier, inlier_midian_dist, 
			matching_draw);

//		result_to_inlier = to_inlier.clone(); //yn

		inlier_rate = (double)from_inlier.rows / (double)from.rows;

		if(!matching_draw.empty())
		{
			cv::putText(matching_draw, 
				cv::format("iter: %d x: %.5lf y: %.5lf th: %.5lf", 
					iter, output.x, output.y, output.th), 
				cv::Point(15, 25), 0, 0.5, cv::Scalar(0, 0, 0)
			);

			cv::putText(matching_draw, cv::format("midian: %10lf ratio: %5.3lf", 
					inlier_midian_dist, inlier_rate),
				cv::Point(15, 55), 0, 0.5, cv::Scalar(0, 0, 0));


			if(!img_log_path.empty()) 
			{
				mkdir(img_log_path.c_str(), 0777);
				cv::imwrite(img_log_path+cv::format("%lf.jpg", 
					ros::Time::now().toSec()), matching_draw);
			}
		}

		if(from_inlier.rows <= 10 || to_inlier.rows <= 10)
		{
			inlier_rate = 0.0;
			break;
		}

		if(iter>=10 && inlier_rate < threashold_low_ratio)
		{
			ROS_WARN("Low matching ratio: %lf", inlier_rate);
			inlier_rate = 0.0;
			break;
		}

		if(iter>=30 && inlier_midian_dist > threashold_inlier_midian_dist)
		{
			ROS_WARN("Cannot find solution");
			inlier_rate = 0.0;
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

	from = points.clone();

	if(inlier_rate < threashold_low_ratio) output_confidence = 0.0;
	else if(inlier_midian_dist > threashold_inlier_midian_dist) output_confidence = 0.0;
	else output_confidence = 1.0;

	if(draw_result)
	{
		draw = matching_draw.clone();

		if(output_confidence == 0.0)
			cv::rectangle(draw, cv::Rect(0, 0, draw.cols, draw.rows), 
				cv::Scalar(0, 0, 255), 5
			);
	}

	return output_confidence;
}