#ifndef MOUSEHANDLER_H
#define MOUSEHANDLER_H


#include <opencv2/opencv.hpp>
#include <vector>

extern std::vector<cv::Point2f> g_clickedPoints;

void mouseCallback(int event, int x, int y, int flags, void* userdata);

#endif // MOUSEHANDLER_H