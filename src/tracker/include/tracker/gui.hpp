#include "tracker/MouseInterface.hpp"
#ifndef GUI_H
#define GUI_H
namespace allen
{
    class button
    {
    public:
        std::string name;
        cv::Rect rect;
        cv::Point button_pt, str_pt;
        bool clicked;
        cv::Mat ROI, ROI_clicked, canvas_basic, canvas_clicked;
        cv::Scalar init_scalar, clicked_scalar;
        float font_scale;
        int b_width, b_height;

    public:
        button(std::string _name, cv::Scalar _scalar)    :
            name(_name),
            clicked(false),
            init_scalar(_scalar), 
            font_scale(1.5f),
            b_width(170), b_height(50)
        {
            clicked_scalar = cv::Scalar(0, 255, 0);
            crop_initRect();
            add_Text();
        }
        button()
        {}
        ~button()
        {}

        void add_Text()
        {
            std::string str = this->name;
            int fontface = cv::FONT_HERSHEY_PLAIN | cv::FONT_ITALIC;
            cv::Scalar scalar = this->init_scalar;
            float scale = this->font_scale;
            int size_name = str.size();

            float shift_ratio = size_name * 0.07f;
            cv::Point pt;
            pt.x = this->rect.x + this->rect.width / 2;
            pt.x -= this->rect.width / 2 * shift_ratio;
            pt.y = this->rect.y + this->rect.height / 2 + 8;
            this->str_pt = pt;

            cv::putText(this->canvas_basic, str, pt, fontface, scale, scalar, 2);
            cv::putText(this->canvas_clicked, str, pt, fontface, scale, scalar, 2);

            this->ROI = canvas_basic(this->rect);
            this->ROI_clicked = canvas_clicked(this->rect);

            //cv::imshow(this->name, this->canvas_basic);    //
            //cv::imshow(this->name + "clicked", this->canvas_clicked);    //
            //cv::imshow(this->name + "roi", this->ROI);
            //cv::imshow(this->name + "roi_c", this->ROI_clicked);
        }
        void crop_initRect()
        {
            cv::Mat _canvas = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0,0,0));
            cv::Mat tmp_canvas = _canvas.clone();
            cv::Mat tmp_canvas_clicked = _canvas.clone();

            int margin = 10;
            cv::Point tmp_pt;
            tmp_pt.x = tmp_canvas.cols / 2; 
            tmp_pt.y = tmp_canvas.rows / 2;
            cv::Rect tmp_rect(tmp_pt.x, tmp_pt.y, this->b_width, this->b_height);
            cv::rectangle(tmp_canvas, tmp_rect, this->init_scalar, 2);
            cv::rectangle(tmp_canvas_clicked, tmp_rect, this->clicked_scalar, 6);

            this->canvas_basic = tmp_canvas.clone();
            this->canvas_clicked = tmp_canvas_clicked.clone();
            this->rect = tmp_rect;
        }
    };
    class GUI
    {
    private:
        cv::Size canvas_s;
        cv::Point base_pt;

    public:
        cv::Mat canvas;
        std::string canvas_win;
        //MouseInterface mi = MouseInterface(1);
        MouseInterface mi;
        button b_init, b_calib;
        bool initialize;

    private:
        void add_button(cv::Mat &_canvas, button &_button, cv::Point _base_pt)
        {
            cv::Point front, tail;
            front.x = _base_pt.x;
            front.y = _base_pt.y;
            tail.x = _base_pt.x + _button.rect.width;
            tail.y = _base_pt.y + _button.rect.height;

            //printf("canvas-> [r: %d], [c: %d]\n", _canvas.rows, _canvas.cols);
            //printf("ROI-> [r: %d][c: %d]\n", _button.ROI.rows, _button.ROI.cols);
            //printf("f[%d, %d], t[%d, %d]\n\n", front.x, front.y, tail.x, tail.y);

            _button.ROI.copyTo(_canvas.rowRange(front.y, tail.y).colRange(front.x, tail.x));
            _button.button_pt = _base_pt;
            _button.rect = cv::Rect(_base_pt.x, _base_pt.y, _button.rect.width, _button.rect.height);
        }
        void init_Canvas(cv::Size &_size)
        {
            if(_size.height <= 0)
            {
                ROS_ERROR("[gui]Size of canvas is invalid..[%d][%d]", _size.height, _size.width);
                this->initialize = false;
                return;
            }
            canvas = cv::Mat(_size, CV_8UC3, cv::Scalar(0,0,0));

            int margin = 20;
            this->base_pt.x = canvas.cols - margin - b_init.rect.width;
            this->base_pt.y = canvas.rows - margin - b_init.rect.height;
            if(this->base_pt.x < 0 || this->base_pt.y < 0)
            {
                ROS_ERROR("[gui]base_pt is out of range..");
                this->initialize = false;
                return;
            }
            add_button(canvas, b_init, this->base_pt);

            this->base_pt.x -= margin + b_calib.rect.width;
            if(this->base_pt.x < 0 || this->base_pt.y < 0)
            {
                ROS_ERROR("[gui]base_pt is out of range..");
                this->initialize = false;
                return;
            }
            add_button(canvas, b_calib, this->base_pt);

            cv::namedWindow(canvas_win);
            mi.setWindow(canvas_win.c_str());

            this->initialize = true;
            ROS_INFO("[GUI]canvas is successfully initialized! [%d][%d]", canvas.rows, canvas.cols);
        }

    public:
        GUI(cv::Size &_size)    :
            canvas_s(_size),
            canvas_win("tracker"),
            initialize(false)
        {
            mi = MouseInterface(1);
            b_init = button("Initialize", cv::Scalar(255,255,255));
            b_calib = button("Calibration", cv::Scalar(255,255,255));
            init_Canvas(canvas_s);
        }
        GUI(){};
        ~GUI()
        {}
        void clicked_button(cv::Mat &_canvas, button &_button)
        {
            if(!this->initialize)   return;

            _button.clicked = !_button.clicked;     //change status of button

            cv::Mat tmp_ROI;
            if(_button.clicked)     tmp_ROI = _button.ROI_clicked.clone();
            else                    tmp_ROI = _button.ROI.clone();

            cv::Point front, tail;
            front.x = _button.button_pt.x;
            front.y = _button.button_pt.y;
            tail.x = _button.button_pt.x + _button.rect.width;
            tail.y = _button.button_pt.y + _button.rect.height;

            if(tmp_ROI.empty())     
            {
                ROS_ERROR("[gui]tmp_ROI is empty..");
                return;
            }

            tmp_ROI.copyTo(_canvas.rowRange(front.y, tail.y).colRange(front.x, tail.x));
            ROS_INFO("[gui]->[%s]button is clicked![%d]", _button.name.c_str(), _button.clicked);
        }
    };

}
#endif


