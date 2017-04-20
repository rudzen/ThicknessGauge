#pragma once

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "_cv.h"

using namespace std;
using namespace _cv;
using namespace cv;

class MiniCalc
{

private:
	Point2i lowestPixel;
	Point2i highestPixel;

	map<Quantile, double> quantileMap = { { Quantile::Q25 , 0.25 },{ Quantile::Q50, 0.5 },{ Quantile::Q75, 0.75 } };

	Point2d m_Mean;

	Point2d m_Variance;

public:

	struct pixelYsort {
		bool operator() (Point2i pt1, Point2i pt2) const { return pt1.y < pt2.y; }
		bool operator() (Point2d pt1, Point2d pt2) const { return pt1.y < pt2.y; }
	} sortY;

	struct pixelXsort {
		bool operator() (Point2i pt1, Point2i pt2) const { return pt1.x < pt2.x; }
		bool operator() (Point2d pt1, Point2d pt2) const { return pt1.x < pt2.x; }
	} sortX;

public:
	MiniCalc();
	~MiniCalc();

	// means of all Y values for each X
	vector<Point> m_PlanarPixels;

	void SetMean(Point2d newMean);

	Point2d GetMean() const;

	void AddMean(Point2d meanToAdd);

	void SetVariance(Point2d newVariance);

	Point2d GetVariance() const;

	void AddVariance(Point2d varianceToAdd);

	static Point2d mean(vector<Point2i> &pixels);

	static double meanX(vector<Point2i> &pixels);

	static double meanY(vector<Point2i> &pixels);

	Point2i quantileX(Quantile quant, vector<Point2i>& pixels);

	Point2i quantileY(Quantile quant, vector<Point2i>& pixels);

	Point2i percentileX(double percentage, vector<Point2i> &pixels) const;

	Point2i percentileY(double percentage, vector<Point2i> &pixels) const;

	Point2d variance(vector<Point2i>& pixels) const;

	Point2d variance(Point2d& mean, vector<Point2i>& pixels) const;

	static void varianceAdd(vector<Point2i> &pixels);

	double varianceX(vector<Point2i> &pixels) const;

	double varianceX(double mean, vector<Point2i> &pixels) const;

	double varianceY(vector<Point2i> &pixels) const;

	double varianceY(double mean, vector<Point2i> &pixels) const;

	Point2d sd(vector<Point2i> &pixels) {
		return Point2d(sqrt(varianceX(pixels)), sqrt(varianceY(pixels)));
	}

	double sdX(vector<Point2i> &pixels) {
		return sqrt(varianceX(pixels));
	}

	double sdY(vector<Point2i> &pixels) {
		return sqrt(varianceY(pixels));
	}

	static double varianceCoefficient(double *__restrict s, int mean) {
		return *s / mean * 100;
	}

	static int calculateHighLow(vector<Point2i> &pixels);

	int highestPixelInLine(Mat &image) const;

	bool saveData(string filename) const;

	/* Converts pixels from an image to a singular line vector in X, where Y is the mean of all pixels located at that given X in the image */
	bool generatePlanarPixels(Mat &input, Mat &output, vector<Point> &pixels) const;

	bool generateGradientPlanarMap(Mat &image, vector<Point2d> planarPixels, vector<Vec3b> &gradientPixels);

	uchar getGradientYValues(Mat & image, int x, int y, int maxY, int minY);

};

