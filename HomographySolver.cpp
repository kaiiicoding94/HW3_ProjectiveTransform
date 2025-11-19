#include "HomographySolver.h"
#include "GaussianElimination.h"
#include <stdexcept>
#include <iostream>
#include <vector>
#include <algorithm>

// Sorts 4 points into a canonical order: top-left, top-right, bottom-right, bottom-left
std::vector<cv::Point2f> sortPoints(const std::vector<cv::Point2f>& points) {
	if (points.size() != 4) {
		throw std::runtime_error("sortPoints requires exactly 4 points.");
	}

	std::vector<cv::Point2f> sorted_points(4);
	std::vector<float> sums;
	std::vector<float> diffs;

	for (const auto& p : points) {
		sums.push_back(p.x + p.y);
		diffs.push_back(p.y - p.x);
	}

	// Top-left has the smallest sum
	int tl_idx = std::distance(sums.begin(), std::min_element(sums.begin(), sums.end()));
	// Bottom-right has the largest sum
	int br_idx = std::distance(sums.begin(), std::max_element(sums.begin(), sums.end()));
	// Top-right has the smallest difference
	int tr_idx = std::distance(diffs.begin(), std::min_element(diffs.begin(), diffs.end()));
	// Bottom-left has the largest difference
	int bl_idx = std::distance(diffs.begin(), std::max_element(diffs.begin(), diffs.end()));

	sorted_points[0] = points[tl_idx]; // Top-Left
	sorted_points[1] = points[tr_idx]; // Top-Right
	sorted_points[2] = points[br_idx]; // Bottom-Right
	sorted_points[3] = points[bl_idx]; // Bottom-Left

	return sorted_points;
}

cv::Mat solveHomography(
	const std::vector<cv::Point2f>& src,
	const std::vector<cv::Point2f>& dst)
{
	if (src.size() != 4 || dst.size() != 4) {
		throw std::runtime_error("solveHomography requires exactly 4 pairs of points.");
	}

	std::vector<cv::Point2f> sorted_src = sortPoints(src);

	// A is 8x8
	std::vector<std::vector<double>> A(8, std::vector<double>(8, 0.0));
	// b is 8x1
	std::vector<double> b(8);

	for (int i = 0; i < 4; i++) {
		double x = sorted_src[i].x;
		double y = sorted_src[i].y;
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
