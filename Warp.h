#ifndef WARP_H
#define WARP_H

#include <opencv2/opencv.hpp>

// 雙線性插值
uchar bilinearInterpolation(const cv::Mat& img, double x, double y);

// 反向映射 + 套用 homography
cv::Mat warpPerspectiveCustom(
    const cv::Mat& src,
    const cv::Mat& Hinv,
    int outWidth,
    int outHeight
);

#endif
