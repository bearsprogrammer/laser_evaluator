#include "calibrator/matcher.hpp"

void matcher::add_pointcloud(bag_t &_src)
{
    if((int)_src.size() == 0)
    {
        ROS_ERROR("[matcher]Empty pointcloud...");
        return;
    }

    bag_cloud_.push_back(_src);
}
bool matcher::checkSync(std::vector<bag_t> &_cloud)
{
    return true;
}
void matcher::runLoop()
{
    ros::Rate r(15);
    while (ros::ok())
    {
        //if(checkSync() == false)
            //continue;

        /* code */
        ros::spinOnce();
        r.sleep();
    }
    
}