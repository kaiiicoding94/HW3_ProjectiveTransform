#include "Warp.h"
#include <cmath>
#include <iostream>

// ------------------------------------------------------------
// 1) Bilinear Interpolation for 3-Channel Color Image
// ------------------------------------------------------------
cv::Vec3b bilinearInterpolation(const cv::Mat& img, double x, double y)
{
    int x0 = (int)std::floor(x);
    int y0 = (int)std::floor(y);

    // Check if the coordinate is out of bounds
    if (x0 < 0 || x0 + 1 >= img.cols || y0 < 0 || y0 + 1 >= img.rows) {
        return cv::Vec3b(0, 0, 0); // Return black for out-of-bounds pixels
    }

    double u = x - x0;
    double v = y - y0;

    // Get the four corner pixels
    const cv::Vec3b& p00 = img.at<cv::Vec3b>(y0, x0);
    const cv::Vec3b& p10 = img.at<cv::Vec3b>(y0, x0 + 1);
    const cv::Vec3b& p01 = img.at<cv::Vec3b>(y0 + 1, x0);
    const cv::Vec3b& p11 = img.at<cv::Vec3b>(y0 + 1, x0 + 1);

    cv::Vec3b result;
    for (int i = 0; i < 3; i++) // Iterate over B, G, R channels
    {
        double interpolated_value =
            (1 - u) * (1 - v) * p00[i] +
            u * (1 - v) * p10[i] +
            (1 - u) * v * p01[i] +
            u * v * p11[i];
        result[i] = (uchar)std::round(interpolated_value);
    }

    return result;
}


// ------------------------------------------------------------
// 2) Backward Mapping using H inverse for 3-Channel Color Image
//    Hinv is a 3x3 double matrix
// ------------------------------------------------------------
cv::Mat warpPerspectiveCustom(
    const cv::Mat& src,
    const cv::Mat& Hinv,
    int outWidth,
    int outHeight)
{
    // Create a 3-channel color output image
    cv::Mat output(outHeight, outWidth, CV_8UC3, cv::Scalar(0, 0, 0));

    // Iterate over each pixel of the output image
    for (int y = 0; y < outHeight; y++) {
        for (int x = 0; x < outWidth; x++) {
            // Apply inverse homography to find the corresponding source coordinate
            double X = Hinv.at<double>(0, 0) * x +
                       Hinv.at<double>(0, 1) * y +
                       Hinv.at<double>(0, 2);

            double Y = Hinv.at<double>(1, 0) * x +
                       Hinv.at<double>(1, 1) * y +
                       Hinv.at<double>(1, 2);

            double Z = Hinv.at<double>(2, 0) * x +
                       Hinv.at<double>(2, 1) * y +
                       Hinv.at<double>(2, 2);

            if (Z == 0) continue; // Avoid division by zero

            double srcX = X / Z;
            double srcY = Y / Z;

            // Get the pixel value using bilinear interpolation
            cv::Vec3b value = bilinearInterpolation(src, srcX, srcY);
            output.at<cv::Vec3b>(y, x) = value;
        }
    }

    return output;
}