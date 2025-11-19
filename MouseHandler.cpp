#include "MouseHandler.h"
#include <iostream>

std::vector<cv::Point2f> g_clickedPoints;

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        g_clickedPoints.emplace_back((float)x, (float)y);
        std::cout << "Clicked point: (" << x << ", " << y << ")" << std::endl;
    }
}