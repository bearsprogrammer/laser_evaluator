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
}

#endif