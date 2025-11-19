#ifndef HOMOGRAPHY_SOLVER_H
#define HOMOGRAPHY_SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat solveHomography(
    const std::vector<cv::Point2f>& src,
    const std::vector<cv::Point2f>& dst
);

#endif
