#ifndef LaserPointCloud_H
#define LaserPointCloud_H
namespace allen
{
    class LaserPointCloud
    { 
    public:
        ros::Time laser_stamp_;
        float angle;
        cv::Point2f laser_coordinate_;

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
    public:
        Target()    :
            target_idx(0)
        {

        }
        ~Target()
        {}
    };


}

#endif