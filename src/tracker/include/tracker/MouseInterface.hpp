#include <iostream>
#include <opencv2/opencv.hpp>

#ifndef MOUSEINTERFACE_H
#define MOUSEINTERFACE_H
class MouseInterface
{
public:
    struct mouse_info
    {
        int x = 0, y = 0;
        bool down = false;
        bool up = false;
        bool drag = false;
        int pre_event = 0;
    };

private:
    int number_of_mouse;
    bool avaliable_touch = false;
    mouse_info* Mouse;

public:
    MouseInterface(int maxMouse_num)
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
    ~MouseInterface()
	{
		delete[] Mouse;
	}

    void setWindow(const char* name, int mouse_idx = 0)
	{
		if (mouse_idx<0 || mouse_idx>this->number_of_mouse - 1)
		{
			printf("There is no %d mouse\n", mouse_idx);
			return;
		}

		cv::setMouseCallback(name, onMouse, &this->Mouse[mouse_idx]);
	}
    bool getDown(int mouse_idx = 0)
    {
        bool val = Mouse[mouse_idx].down;

        if (val) Mouse[mouse_idx].down = false;

        return val;
    };
    bool getUp(int mouse_idx = 0)
    {
        bool val = Mouse[mouse_idx].up;

        if (val) Mouse[mouse_idx].up = false;

        return val;
    };
    bool getDrag(int mouse_idx = 0)
    {
        return Mouse[mouse_idx].drag;
    };
    int getX(int mouse_idx = 0)
    {
        return Mouse[mouse_idx].x;
    }
    int getY(int mouse_idx = 0)
    {
        return Mouse[mouse_idx].y;
    }
};
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
#endif