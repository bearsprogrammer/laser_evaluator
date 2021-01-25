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
        float target_radius;

    public:
        Target()    :
            target_idx(0)
        {

        }
        ~Target()
        {}
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
    };


}

#endif

