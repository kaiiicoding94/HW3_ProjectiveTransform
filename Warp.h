#ifndef WARP_H
#define WARP_H

#include <opencv2/opencv.hpp>

// ���u�ʴ���
cv::Vec3b bilinearInterpolation(const cv::Mat& img, double x, double y);

// �ϦV�M�g + �M�� homography
cv::Mat warpPerspectiveCustom(
    const cv::Mat& src,
    const cv::Mat& Hinv,
    int outWidth,
    int outHeight
);

#endif
