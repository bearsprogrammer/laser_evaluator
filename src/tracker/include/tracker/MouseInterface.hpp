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
    MouseInterface(int maxMouse_num);
    MouseInterface()
    {}
    ~MouseInterface();

    void setWindow(const char* name, int mouse_idx = 0);
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
#endif