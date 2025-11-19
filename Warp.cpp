#include "Warp.h"
#include <cmath>
#include <iostream>

// ------------------------------------------------------------
// 1) Bilinear Interpolation
// ------------------------------------------------------------
uchar bilinearInterpolation(const cv::Mat& img, double x, double y)
{
	int x0 = (int)std::floor(x);
	int y0 = (int)std::floor(y);

	// 超出邊界 → 回傳黑色
	if (x0 < 0 || x0 + 1 >= img.cols ||
		y0 < 0 || y0 + 1 >= img.rows) {
		return 0;
	}

	double u = x - x0;
	double v = y - y0;

	// 取得四個鄰近像素
	uchar p00 = img.at<uchar>(y0, x0);
	uchar p10 = img.at<uchar>(y0, x0 + 1);
	uchar p01 = img.at<uchar>(y0 + 1, x0);
	uchar p11 = img.at<uchar>(y0 + 1, x0 + 1);

	// 雙線性插值
	double result =
		(1 - u) * (1 - v) * p00 +
		u * (1 - v) * p10 +
		(1 - u) * v * p01 +
		u * v * p11;

	return (uchar)std::round(result);
}


// ------------------------------------------------------------
// 2) Backward Mapping using H inverse
//    Hinv 是 3×3 double matrix
// ------------------------------------------------------------
cv::Mat warpPerspectiveCustom(
	const cv::Mat& src,
	const cv::Mat& Hinv,
	int outWidth,
	int outHeight)
{
	cv::Mat output(outHeight, outWidth, CV_8UC1, cv::Scalar(0));

	// 逐像素填入
	for (int y = 0; y < outHeight; y++) {
		for (int x = 0; x < outWidth; x++) {
			// 反推回原圖座標
			double X = Hinv.at<double>(0, 0) * x +
				Hinv.at<double>(0, 1) * y +
				Hinv.at<double>(0, 2);

			double Y = Hinv.at<double>(1, 0) * x +
				Hinv.at<double>(1, 1) * y +
				Hinv.at<double>(1, 2);

			double Z = Hinv.at<double>(2, 0) * x +
				Hinv.at<double>(2, 1) * y +
				Hinv.at<double>(2, 2);

			if (Z == 0) continue;

			double srcX = X / Z;
			double srcY = Y / Z;

			// 取像素（bilinear）
			uchar value = bilinearInterpolation(src, srcX, srcY);
			output.at<uchar>(y, x) = value;
		}
	}

	return output;
}
