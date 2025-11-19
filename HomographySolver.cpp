#include "HomographySolver.h"
#include "GaussianElimination.h"
#include <stdexcept>
#include <iostream>

cv::Mat solveHomography(
	const std::vector<cv::Point2f>& src,
	const std::vector<cv::Point2f>& dst)
{
	if (src.size() != 4 || dst.size() != 4) {
		throw std::runtime_error("solveHomography requires exactly 4 pairs of points.");
	}

	// A is 8x8
	std::vector<std::vector<double>> A(8, std::vector<double>(8, 0.0));
	// b is 8x1
	std::vector<double> b(8);

	for (int i = 0; i < 4; i++) {
		double x = src[i].x;
		double y = src[i].y;
		double X = dst[i].x;
		double Y = dst[i].y;

		// Row 2*i :   X = (a x + b y + c) / (g x + h y + 1)
		// => x*a + y*b + 1*c + 0*d + 0*e + 0*f - x*X*g - y*X*h = X
		A[2 * i][0] = x;
		A[2 * i][1] = y;
		A[2 * i][2] = 1;
		A[2 * i][3] = 0;
		A[2 * i][4] = 0;
		A[2 * i][5] = 0;
		A[2 * i][6] = -x * X;
		A[2 * i][7] = -y * X;
		b[2 * i] = X;

		// Row 2*i+1 : Y = (d x + e y + f) / (g x + h y + 1)
		A[2 * i + 1][0] = 0;
		A[2 * i + 1][1] = 0;
		A[2 * i + 1][2] = 0;
		A[2 * i + 1][3] = x;
		A[2 * i + 1][4] = y;
		A[2 * i + 1][5] = 1;
		A[2 * i + 1][6] = -x * Y;
		A[2 * i + 1][7] = -y * Y;
		b[2 * i + 1] = Y;
	}

	// Solve: A * h = b
	std::vector<double> h = gaussianElimination(A, b);

	// Build 3x3 homography matrix H:
	// [a b c]
	// [d e f]
	// [g h 1]

	cv::Mat H = (cv::Mat_<double>(3, 3) <<
		h[0], h[1], h[2],
		h[3], h[4], h[5],
		h[6], h[7], 1.0
		);

	return H;
}
