#ifndef LaserPointCloud_H
#define LaserPointCloud_H
namespace allen
{
    class LaserPointCloud
    { 
    public:
        ros::Time laser_stamp_;
        float angle;
        cv::Point2f laser_coordinate_;      //mm

    public:
        LaserPointCloud()
        {
        }
        ~LaserPointCloud(){}
        void release()
        {

        }
    };
    class Target
    {
    public:
        int target_idx;
        cv::Rect target_rect;
        cv::Point center_pt;
        cv::Point2f centroid_pt;
        float target_radius;
        std::vector<cv::Point2f> object_pts, src_object_pts;
        cv::Mat src_dp_mat;

    public:
        Target()    :
            target_idx(0)
        {
            reset();
        }
        ~Target()
        {}
        void reset()
        {
            target_idx = 0;
            target_rect = cv::Rect();
            center_pt = cv::Point();
            centroid_pt.x = std::numeric_limits<float>::infinity();
            centroid_pt.y = std::numeric_limits<float>::infinity();
            target_radius = std::numeric_limits<float>::infinity();
            object_pts = std::vector<cv::Point2f>();
            src_object_pts = std::vector<cv::Point2f>();
            src_dp_mat = cv::Mat();
        }
        void set_Centerpt()
        {
            if(this->target_rect.width <= 0 || this->target_rect.height <= 0)
            {
                ROS_ERROR("[tracker]target_rect's width[%d], height[%d] are unset..", 
                                        this->target_rect.width, this->target_rect.height);
                return;
            }
            cv::Point tmp_center;
            tmp_center.x = this->target_rect.x + (this->target_rect.width/2);  
            tmp_center.y = this->target_rect.y + (this->target_rect.height/2);  

            if(this->target_rect.contains(tmp_center) == false)
            {
                ROS_ERROR("[tracker]target_rect's center_pt is invalid[%d, %d]..", 
                                        tmp_center.x, tmp_center.y);
                return;
            }

            this->center_pt = tmp_center;
        }
        cv::Rect set_Region(cv::Point _grid_center)
        {
            allen::Grid_param grid;

            float width_mm = 2000.0f;
            float height_mm = 2000.0f;
            float width_pixel = width_mm * grid.mm2pixel;
            float height_pixel = height_mm * grid.mm2pixel;
            cv::Point start_pt;
            start_pt.x = _grid_center.x - width_pixel/2.0f;
            start_pt.y = _grid_center.y - height_pixel/2.0f;

            cv::Rect output_rect(start_pt, cv::Size(width_pixel, height_pixel));
            return output_rect;
        }
        std::vector<cv::Point2f> cvtFloat(std::vector<cv::Point2f> &_src)
        {
            std::vector<cv::Point2f> output;
            for(int i = 0; i < (int)_src.size(); i++)
            {
                cv::Point2f tmp_pt;
                tmp_pt.x = _src[i].x / 1000.0f;
                tmp_pt.y = _src[i].y / 1000.0f;
                output.push_back(tmp_pt);
            }
            return output;
        }
    };


}

#endif

