#include "tracker/MouseInterface.hpp"
#ifndef GUI_H
#define GUI_H
namespace allen
{
    class GUI
    {
    private:
        cv::Size canvas_s;

    public:
        cv::Mat canvas;
        std::string canvas_win;

    public:
        GUI(cv::Size &_size)    :
            canvas_s(_size),
            canvas_win("tracker")
        {
            init_Canvas(canvas, canvas_s);
        }
        GUI(){};
        ~GUI()
        {}
        void init_Canvas(cv::Mat &_canvas, cv::Size &_size)
        {
            if(_size.height <= 0)
            {
                ROS_ERROR("[gui]Size of canvas is invalid..[%d][%d]", _size.height, _size.width);
                return;
            }
            _canvas = cv::Mat(_size, CV_8UC3, cv::Scalar(0,0,0));
            cv::namedWindow(canvas_win);

            ROS_INFO("[GUI]canvas is successfully initialized! [%d][%d]", _canvas.rows, _canvas.cols);
        }
    };

}
#endif