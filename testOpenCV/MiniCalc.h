#pragma once

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "_cv.h"
#include "Interpolate.h"
#include "Bezier_1.h"
#include "LineConfig.h"

using namespace std;
using namespace _cv;
using namespace cv;

class MiniCalc {

private:

	Point2i lowestPixel_;
	Point2i highestPixel_;

	map<Quantile, double> quantileMap_ = {{Quantile::Q25 , 0.25},{Quantile::Q50, 0.5},{Quantile::Q75, 0.75}};

	Point2d mean_;

	Point2d variance_;

	Interpolate<double> interpolate_;

	Bezier<double> bezier_;
	
public:

	struct pixelYsort {
		bool operator()(Point2i pt1, Point2i pt2) const { return pt1.y < pt2.y; }
		bool operator()(Point2d pt1, Point2d pt2) const { return pt1.y < pt2.y; }
	} sortY;

	struct pixelXsort {
		bool operator()(Point2i pt1, Point2i pt2) const { return pt1.x < pt2.x; }
		bool operator()(Point2d pt1, Point2d pt2) const { return pt1.x < pt2.x; }
	} sortX;

	void sortContours(vector<vector<Point>>& contours) {
		auto contourComparator = [](vector<Point> a, vector<Point> b) { return contourArea(a) > contourArea(b); };
		sort(contours.begin(), contours.end(), contourComparator);
	}

public:
	MiniCalc();
	~MiniCalc();

	static double calculatePixelToMm(int pixels);

	// means of all Y values for each X
	vector<Point> m_PlanarPixels;

	void SetMean(Point2d newMean);

	Point2d GetMean() const;

	void AddMean(Point2d meanToAdd);

	void SetVariance(Point2d newVariance);

	Point2d GetVariance() const;

	void AddVariance(Point2d varianceToAdd);

	static Point2d mean(vector<Point2i>& pixels);

	static double meanX(vector<Point2i>& pixels);

	static double meanY(vector<Point2i>& pixels);

	Point2i quantileX(Quantile quant, vector<Point2i>& pixels);

	Point2i quantileY(Quantile quant, vector<Point2i>& pixels);

	Point2i percentileX(double percentage, vector<Point2i>& pixels) const;

	Point2i percentileY(double percentage, vector<Point2i>& pixels) const;

	Point2d variance(vector<Point2i>& pixels) const;

	Point2d variance(Point2d& mean, vector<Point2i>& pixels) const;

	static void varianceAdd(vector<Point2i>& pixels);

	double varianceX(vector<Point2i>& pixels) const;

	double varianceX(double mean, vector<Point2i>& pixels) const;

	double varianceY(vector<Point2i>& pixels) const;

	double varianceY(double mean, vector<Point2i>& pixels) const;

	Point2d sd(vector<Point2i>& pixels) const {
		return Point2d(sqrt(varianceX(pixels)), sqrt(varianceY(pixels)));
	}

	double sdX(vector<Point2i>& pixels) const {
		return sqrt(varianceX(pixels));
	}

	double sdY(vector<Point2i>& pixels) const {
		return sqrt(varianceY(pixels));
	}

	static double varianceCoefficient(double*__restrict s, int mean) {
		return *s / mean * 100;
	}

	static int calculateHighLow(vector<Point2i>& pixels);

	int highestPixelInLine(Mat& image) const;

	bool saveData(string filename) const;

	/* Converts pixels from an image to a singular line vector in X, where Y is the mean of all pixels located at that given X in the image */
	bool generatePlanarPixels(Mat& input, Mat& output, vector<Point>& pixels, vector<Point2d>& gradientPixels) const;

	bool generateGradientPlanarMap(Mat& image, vector<Point2d> planarPixels, vector<Vec3b>& gradientPixels);

	uchar getGradientYValues(Mat& image, int x, int y, int maxY, int minY);

	/**
	 * \brief Computes mu
	 * Simple mu computation helper function for use with Interpolate and Bezier calculations
	 * \param currentPosition The current position
	 * \param inBetweenCount How many points there are in the source position and the target position
	 * \return The computed mu
	 */
	static double computeMu(double currentPosition, double inBetweenCount) {
		return currentPosition / inBetweenCount;
	}

	/**
	 * \brief Computes lines between two points
	 * \param currentPoint The source point
	 * \param targetPoint The destination point
	 * \param result The resulting vector of points which makes up the line
	 * \return true if the result is not empty, otherwise false
	 */
	static bool computeSimpleLine(cv::Point& currentPoint, cv::Point& targetPoint, vi& result) {

	}

	cv::Point getRightEdge(vi& elements, int imageHeight, int baseLevel) const {

		sort(elements.begin(), elements.end(), sortX);

		const auto barrier = imageHeight - baseLevel;

		const auto size = elements.size() / 2;

		for (auto i = size; i > 0; --i) {
			if (elements[i].y < barrier)
				return elements[i];
		}
		
		return Point(0, imageHeight);
	}

	cv::Point getLeftEdge(vi& elements, int imageHeight, int baseLevel) const {

		sort(elements.begin(), elements.end(), sortX);

		const auto barrier = imageHeight - baseLevel;

		const auto size = elements.size() / 2;

		for (auto i = size; i > 0; ++i) {
			if (elements[i].y < barrier)
				return elements[i];
		}

		return Point(0, imageHeight);
	}

	void computeLeftElements(vi& elements, vi& targetElements, int leftEdge, int imageWidth, bool sortElements) const {

		if (sortElements)
			sort(elements.begin(), elements.end(), sortX);

		targetElements.clear();

		for (auto& e : elements) {
			if (e.x >= leftEdge)
				targetElements.push_back(e);
		}

		// just fill out the rest towards the left of the border
		if (targetElements.back().x < imageWidth) {
			const auto targetLastX = targetElements.back().x;
			const auto targetLastY = targetElements.back().y;
			for (auto i = targetLastX; i < imageWidth; ++i)
				targetElements.push_back(Point(i, targetLastY));
		}
	}

	void computeRightElements(vi& elements, vi& targetElements, int rightEdge, bool sortElements) const {
		
		if (sortElements)
			sort(elements.begin(), elements.end(), sortX);

		targetElements.clear();

		for (auto& e : elements) {
			if (e.x > rightEdge)
				break;
			targetElements.push_back(e);
		}

		// just fill out the rest towards the left of the border
		if (targetElements.back().x > 0) {
			const auto targetLastX = targetElements.back().x;
			const auto targetLastY = targetElements.back().y;
			for (auto i = targetLastX; i >= 0; --i)
				targetElements.push_back(Point(i, targetLastY));
		}

	}


	bool fillElementGabs(vi& elements, cv::Mat& target, int barrier) const {

		auto gab = false;
		auto result = false;

		elements.push_back(Point(0, target.rows));
		elements.push_back(Point(target.cols, target.rows));

		sort(elements.begin(), elements.end(), sortX);

		auto size = elements.size();

		elements.push_back(elements.front());

		auto first = elements.front().x;

		vector<Point> gabLine;

		for (auto i = first; i < size; ++i) {
			//if (elements[i].y > barrier)
			//	continue;
			if (i + 1 == size)
				break;
			auto posX = elements[i].x;
			auto nextX = elements[i + 1].x;
			auto dif = nextX - posX;
			if (dif < 2) {
				// no gab in x, try with y !!
				auto posY = elements[i].y;
				auto nextY = elements[i + 1].y;
				auto diffY = nextY - posY;
				if (diffY < 4)
					continue;
				gab = true;
			}
			if (gab) {
				line(target, elements[i], elements[i + 1], Scalar(255, 255, 255), 1, LINE_AA);
				result = true;
			}
			gab ^= true;
		}


		//if (gabLine.empty())
		//	return false;

		//cout << "gab detected.." << gabLine.size() << endl;

		//elements = gabLine;

		// remove temporary added element
		//elements.erase(elements.end());
		return result;
	}

	static double computeYSum(vi& elements) {
		auto sum = 0.0;
		for (auto& e : elements)
			sum += e.y;
		return sum;
	}

	double computeWeigthedY(vi& elements, int x) {

		vi temp;

		getElementsX(elements, temp, x);

		if (temp.empty())
			return 0.0;

		return computeYSum(temp) / static_cast<double>(temp.size());

	}

	double computeWeigthedIntensity(vi& elements, int x, int y) {
		return 0.0;
	}

	double computeWeigthedIntensity(vi& elements, Point point) {
		return computeWeigthedIntensity(elements, point.x, point.y);
	}

	static void getElementsX(vi& input, vi& output, int x) {
		for (auto& e : input) {
			if (e.x == x)
				output.push_back(e);
		}
	}

	static void getElementsY(vi& input, vi& output, int y) {
		for (auto& e : input) {
			if (e.y == y)
				output.push_back(e);
		}
	}

	/**
	 * \brief Computes a line from a vector of pixels points
	 * \param pixels The pixels to calculate the line from
	 * \param result The resulting line
	 * \return true if line was created, otherwise false
	 */
	static bool computerCompleteLine(vi& pixels, cv::Vec4f& result, LineConfig& config) {
		cv::fitLine(pixels, result, config.getDistType(), config.getParams(), config.getReps(), config.getAepa());
		return true;
	}


	/**
	 * \brief Crude cutoff of pixels from image based on Y
	 * \param image The image data
	 * \param output The output vector
	 * \param yLimit The limit in height
	 * \return true if something was found, otherwise false
	 */
	static bool getActualPixels(Mat& image, vi& output, int yLimit) {
		vi result;
		cv::findNonZero(image, result);
		yLimit = abs(image.rows - yLimit);
		for (auto& p : result) {
			if (p.y <= yLimit)
				output.push_back(p);
		}
		return !output.empty();
	}

	static bool getActualPixels(vi& pixels, vi&target, int yLimit, int imageHeight) {
		if (!target.empty())
			target.clear();

		yLimit = abs(imageHeight - yLimit);

		for (auto& p : pixels) {
			if (p.y <= yLimit)
				target.push_back(p);
		}

		return !target.empty();
	}



};
