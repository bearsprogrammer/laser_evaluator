#ifndef KCF_LASER_H
#define KCF_LASER_H

class KCF_laser
{
private:
    /* data */

public:
    cv::Ptr<cv::Tracker> tracker_KCF;
    cv::Rect2d roi_kcf;
    int target_idx;

public:
    KCF_laser(int _target_idx)  :
        target_idx(_target_idx)
    {
        tracker_KCF = cv::TrackerKCF::create();
        ROS_INFO("KCF tracker-[%d] is created!", target_idx);
    }

    cv::Point get_Centerpt(cv::Rect2d &_src)
    {
        cv::Point output;
        if(_src.width <= 0 || _src.height <= 0)
        {
            ROS_ERROR("[KCF]target_rect's width[%lf], height[%lf] are unset..", 
                                    _src.width, _src.height);
            output.x = -1;
            output.y = -1;
            return output;
        }
        output.x = _src.x + (_src.width/2);  
        output.y = _src.y + (_src.height/2);  

        return output;
    }
};


#endif