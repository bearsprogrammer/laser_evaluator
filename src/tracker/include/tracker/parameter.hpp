#ifndef PARAMETER_H
#define PARAMETER_H

#include <ros/ros.h>
namespace allen
{
    class Grid_param
    {
    public:
        const int grid_row, grid_col;
        int robot_col, robot_row;
        std::vector<cv::Point2f> base_pt;
        float mm2pixel;
        cv::Mat occup, free;

    public:
        Grid_param() : grid_row(1000), grid_col(1000)
        {
            robot_col = grid_col / 2;
            robot_row = grid_row / 2;
            mm2pixel = 50.0f / 1000.0f;     //1000mm(1m) -> 50 pixel, 20mm -> 1 pixel
            occup = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
            free = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
        }
        ~Grid_param()
        {}
    };

    class Frame
    {
    public:
        double x, y, th;

    public:
        void rearrange_Angle(void)
        {
            while(1)
            {
                if(this->th < -180.0)       this->th += 360.0;
                else if(this->th > 180.0)   this->th -= 360.0;
                else break;
            }
        }
        Frame(void) :
            x(0.0), y(0.0), th(0.0)
        {
        }
        Frame(double _x, double _y, double _th) :
            x(_x), y(_y), th(_th)
        {
        }
        void translate(cv::Mat &_src, cv::Mat &_dst)
        {
            if(_src.type() != 5)
            {
                ROS_ERROR( 
                    "[Frame]-> allen::Frame translate error. type is not 5 %d", _src.type());
                return;
            }	
            if(_src.cols != 2)
            {
                ROS_ERROR( 
                    "[Frame]-> allen::Frame translate error. rows is not 2 %d", _src.rows);
                return;
            }	

            if(_src.size() != _dst.size())
            {
                _dst = cv::Mat(_src.size(), CV_32FC1);
            }

            double base_radian_th = M_PI/180.0 * this->th;
            for(int i=0; i<_src.rows; i++)
            {
                float t_x = _src.at<float>(i, 0);	
                float t_y = _src.at<float>(i, 1);	
                _dst.at<float>(i, 0) 
                    = this->x + t_x*cos(base_radian_th) - t_y*sin(base_radian_th);
                _dst.at<float>(i, 1) 
                    = this->y + t_x*sin(base_radian_th) + t_y*cos(base_radian_th);
                //printf("src-> [x: %f, y: %f], dst-> [x: %f, y: %f]\n", t_x, t_y, 
                                                                    //_dst.at<float>(i, 0), _dst.at<float>(i, 1));
            }
        }
        Frame add_Frame(Frame &_src)
        {
            double d2r = 3.141592 / 180.0;
            double base_radian_th = this->th*d2r;

            Frame output;
            output.x = this->x + _src.x*cos(base_radian_th) - _src.y*sin(base_radian_th);
            output.y = this->y + _src.x*sin(base_radian_th) + _src.y*cos(base_radian_th);
            output.th = this->th + _src.th;
            output.rearrange_Angle();

            return output;
        }

    };
    
}
#endif