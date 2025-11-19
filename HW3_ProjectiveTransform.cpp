#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

#include "MouseHandler.h"
#include "HomographySolver.h"
#include "Warp.h"

using namespace std;
using namespace cv;

int main()
{
	// -------------------------------------------------
	// 1. Load image
	// -------------------------------------------------
	string imgPath;
	cout << "Please enter the path to the image file: ";
	cin >> imgPath;
	Mat src = imread(imgPath, IMREAD_COLOR);

	if (src.empty())
	{
		cerr << "Error: Cannot load image." << endl;
		return -1;
	}

	cout << "Image loaded. Size = "
		<< src.cols << " x " << src.rows << endl;

	// -------------------------------------------------
	// 2. Let user click 4 points
	// -------------------------------------------------
	namedWindow("Select 4 Points", WINDOW_AUTOSIZE);
	setMouseCallback("Select 4 Points", mouseCallback, nullptr);

	cout << "Please click 4 points in order: " << endl;
	cout << "1: Top-left" << endl;
	cout << "2: Top-right" << endl;
	cout << "3: Bottom-right" << endl;
	cout << "4: Bottom-left" << endl;

	while (true)
	{
		imshow("Select 4 Points", src);

		if (g_clickedPoints.size() >= 4)
		{
			cout << "4 points collected." << endl;
			break;
		}

		int key = waitKey(10);
		if (key == 27) // ESC
		{
			cout << "User cancelled." << endl;
			return 0;
		}
	}
	destroyWindow("Select 4 Points");

	// Store the clicked points locally
	vector<Point2f> srcPts = g_clickedPoints;

	cout << "\nSelected Points:" << endl;
	for (int i = 0; i < 4; i++)
	{
		cout << i << ": (" << srcPts[i].x << ", " << srcPts[i].y << ")" << endl;
	}

	// -------------------------------------------------
	// 3. Compute the output rectangle size automatically
	// -------------------------------------------------
	auto distance2D = [](Point2f a, Point2f b)
		{
			return sqrt((a.x - b.x) * (a.x - b.x) +
				(a.y - b.y) * (a.y - b.y));
		};

	double top = distance2D(srcPts[0], srcPts[1]);
	double bottom = distance2D(srcPts[3], srcPts[2]);
	double left = distance2D(srcPts[0], srcPts[3]);
	double right = distance2D(srcPts[1], srcPts[2]);

	int outWidth = (int)round(max(top, bottom));
	int outHeight = (int)round(max(left, right));

	cout << "\nOutput Size: " << outWidth << " x " << outHeight << endl;

	// -------------------------------------------------
	// 4. Create destination points (perfect rectangle)
	// -------------------------------------------------
	vector<Point2f> dstPts(4);
	dstPts[0] = Point2f(0, 0);
	dstPts[1] = Point2f(outWidth - 1, 0);
	dstPts[2] = Point2f(outWidth - 1, outHeight - 1);
	dstPts[3] = Point2f(0, outHeight - 1);

	// -------------------------------------------------
	// 5. Solve Homography H (custom implementation)
	// -------------------------------------------------
	Mat H = solveHomography(srcPts, dstPts);

	cout << "\nHomography Matrix H:" << endl << H << endl;

	// -------------------------------------------------
	// 6. Compute inverse of H
	// -------------------------------------------------
	Mat Hinv = H.inv();
	cout << "\nInverse Homography Hinv:" << endl << Hinv << endl;

	// -------------------------------------------------
	// 7. Warp using BACKWARD mapping + bilinear
	// -------------------------------------------------
	Mat output = warpPerspectiveCustom(src, Hinv, outWidth, outHeight);

	// -------------------------------------------------
	// 8. Save output
	// -------------------------------------------------
	imwrite("output.png", output);
	cout << "\nSaved result to output.png" << endl;

	// Show result
	namedWindow("Result", WINDOW_AUTOSIZE);
	imshow("Result", output);
	waitKey(0);

	return 0;
}
