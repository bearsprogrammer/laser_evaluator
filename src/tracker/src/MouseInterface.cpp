#include "tracker/MouseInterface.hpp"

MouseInterface::MouseInterface(int maxMouse_num = 1)
{
    number_of_mouse = maxMouse_num;
    Mouse = new mouse_info[maxMouse_num];

    std::cout << "Touch screen has not been implemented yet" << std::endl;

    // int val = GetSystemMetrics(SM_DIGITIZER);
    // if (NID_INTEGRATED_TOUCH & val)
    // {
    //     if (NID_READY & val)
    //     {
    //         printf("TOUCH SCREEN is available\n");
    //         avaliable_touch = true;
    //     }
    // }
}

MouseInterface::~MouseInterface()
{
    delete[] Mouse;
}

void onMouse(int event, int x, int y, int, void* param)
{
    MouseInterface::mouse_info *input_mouse = (MouseInterface::mouse_info *)param;

    input_mouse->x = x;
    input_mouse->y = y;

    if (event == 1)
    {
        input_mouse->down = true;
    }

    else if (event == 4)
    {
        input_mouse->up = true;
        input_mouse->drag = false;
    }

    else if (event == 0 && input_mouse->pre_event == 1)
    {
        input_mouse->drag = true;
    }

    input_mouse->pre_event = event;
};

void MouseInterface::setWindow(const char* name, int mouse_idx)
{
    if (mouse_idx<0 || mouse_idx>this->number_of_mouse - 1)
    {
        printf("There is no %d mouse\n", mouse_idx);
        return;
    }

    cv::setMouseCallback(name, onMouse, &this->Mouse[mouse_idx]);
}
