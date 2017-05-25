#pragma once

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "tg.h"
#include "LineConfig.h"
#include "../Util/Vec.h"

using namespace std;
using namespace tg;

class MiniCalc {

private:

	cv::Point2i lowestPixel_;
	cv::Point2i highestPixel_;

	cv::Point2d mean_;

	cv::Point2d variance_;

public:

	struct pixelYsort {
		bool operator()(cv::Point2i pt1, cv::Point2i pt2) const { return pt1.y < pt2.y; }
		bool operator()(cv::Point2d pt1, cv::Point2d pt2) const { return pt1.y < pt2.y; }
		bool operator()(cv::Point2f pt1, cv::Point2f pt2) const { return pt1.y < pt2.y; }
	} sortY;

	struct pixelXsort {
		bool operator()(cv::Point2i pt1, cv::Point2i pt2) const { return pt1.x < pt2.x; }
		bool operator()(cv::Point2d pt1, cv::Point2d pt2) const { return pt1.x < pt2.x; }
		bool operator()(cv::Point2f pt1, cv::Point2f pt2) const { return pt1.x < pt2.x; }
	} sortX;

	void sortContours(vector<vector<cv::Point>>& contours) {
		auto contourComparator = [](vector<cv::Point> a, vector<cv::Point> b) { return contourArea(a) > contourArea(b); };
		sort(contours.begin(), contours.end(), contourComparator);
	}

public:
	MiniCalc();
	~MiniCalc();

	// means of all Y values for each X
	vector<cv::Point> m_PlanarPixels;

	void setMean(cv::Point2d newMean);

	cv::Point2d getMean() const;

	void addMean(cv::Point2d meanToAdd);

	void setVariance(cv::Point2d newVariance);

	cv::Point2d getVariance() const;

	void addVariance(cv::Point2d varianceToAdd);

	static cv::Point2d mean(vector<cv::Point2i>& pixels);

	static double mean(vector<double>& vec);

	static double meanX(vector<cv::Point2i>& pixels);

	static double meanY(vector<cv::Point2i>& pixels);

	cv::Point2i percentileX(double percentage, vector<cv::Point2i>& pixels) const;

	cv::Point2i percentileY(double percentage, vector<cv::Point2i>& pixels) const;

	cv::Point2d variance(vector<cv::Point2i>& pixels) const;

	cv::Point2d variance(cv::Point2d& mean, vector<cv::Point2i>& pixels) const;

	static void varianceAdd(vector<cv::Point2i>& pixels);

	double varianceX(vector<cv::Point2i>& pixels) const;

	double varianceX(double mean, vector<cv::Point2i>& pixels) const;

	double varianceY(vector<cv::Point2i>& pixels) const;

	double varianceY(double mean, vector<cv::Point2i>& pixels) const;

	cv::Point2d sd(vector<cv::Point2i>& pixels) const {
		return cv::Point2d(sqrt(varianceX(pixels)), sqrt(varianceY(pixels)));
	}

	double sdX(vector<cv::Point2i>& pixels) const {
		return sqrt(varianceX(pixels));
	}

	double sdY(vector<cv::Point2i>& pixels) const {
		return sqrt(varianceY(pixels));
	}

	static double varianceCoefficient(double*__restrict s, int mean) {
		return *s / mean * 100;
	}

	static int calculateHighLow(vector<cv::Point2i>& pixels);

	int highestPixelInLine(cv::Mat& image) const;

	bool saveData(string filename) const;

	/* Converts pixels from an image to a singular line vector in X, where Y is the mean of all pixels located at that given X in the image */
	bool generatePlanarPixels(cv::Mat& input, cv::Mat& output, vector<cv::Point2f>& pixels, vector<cv::Point2f>& gradientPixels) const;

	bool generateGradientPlanarMap(cv::Mat& image, vector<cv::Point2d> planarPixels, vector<cv::Vec3b>& gradientPixels);

	uchar getGradientYValues(cv::Mat& image, int x, int y, int maxY, int minY);

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

	/**
	 * \brief Right edge detection\n
	 * Retrieve the right edge of the part to be measured
	 * by looking at the center of the elements and tracking them towards the right edge until
	 * the baseLevel is hit.
	 * \param elements The current data elements
	 * \param imageHeight The image height
	 * \param baseLevel The base level of the entirety of the data
	 * \return The point where the elements to be measured are below baseline
	 */
	cv::Point getRightEdge(vi& elements, int imageHeight, int baseLevel) const {

		sort(elements.begin(), elements.end(), sortX);

		const auto barrier = imageHeight - baseLevel;

		const auto size = elements.size() / 2;

		for (auto i = size; i > 0; --i) {
			if (elements[i].y < barrier)
				return elements[i];
		}
		
		return cv::Point(0, imageHeight);
	}

	/**
	* \brief Left edge detection\n
	* Retrieve the left edge of the part to be measured
	* by looking at the center of the elements and tracking them backwards to the left edge until
	* the baseLevel is hit.
	* \param elements The current data elements
	* \param imageHeight The image height
	* \param baseLevel The base level of the entirety of the data
	* \return The point where the elements to be measured are below baseline
	*/
	cv::Point getLeftEdge(vi& elements, int imageHeight, int baseLevel) const {

		sort(elements.begin(), elements.end(), sortX);

		const auto barrier = imageHeight - baseLevel;

		const auto size = elements.size() / 2;

		for (auto i = size; i > 0; ++i) {
			if (elements[i].y < barrier)
				return elements[i];
		}

		return cv::Point(0, imageHeight);
	}

	/**
	 * \brief Compute the location of the elements on the left side of measuring target
	 * \param elements All current elements
	 * \param targetElements The resulting elements will be stored here
	 * \param leftEdge The left edge
	 * \param imageWidth The width of the image
	 * \param sortElements if true, the source elements will be sorted, otherwise they wont
	 */
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
				targetElements.push_back(cv::Point(i, targetLastY));
		}
	}

	/**
	* \brief Compute the location of the elements on the right side of measuring target
	* \param elements All current elements
	* \param targetElements The resulting elements will be stored here
	* \param rightEdge The right edge
	* \param sortElements if true, the source elements will be sorted, otherwise they wont
	*/
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
				targetElements.push_back(cv::Point(i, targetLastY));
		}

	}

	/**
	 * \brief Fills the gabs in the elements with simple linear lines
	 * \param elements The elements to fill gabs in
	 * \param target The target matrix
	 * \param barrier The current known location of the baseLevel, default is 0, which means all gabs will be filled
	 * \return Vector (2D) representing the number of elements filled in both X and Y
	 */
	v2<double> fillElementGabs(vi& elements, cv::Mat& target, int barrier = 0) const {

		elements.push_back(cv::Point(0, target.rows));
		elements.push_back(cv::Point(target.cols, target.rows));

		sort(elements.begin(), elements.end(), sortX);

		auto size = elements.size();

		elements.push_back(elements.front());

		auto first = elements.front().x;

		vector<cv::Point> gabLine;

		v2<double> sum;
		auto gab = false;

		for (auto i = first; i < size; ++i) {
			if (elements[i].y > target.rows - barrier)
				continue;
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
				line(target, elements[i], elements[i + 1], cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
				sum.x += abs(elements[i + 1].x - elements[i].x);
				sum.y += abs(elements[i + 1].y - elements[i].y);
			}
			gab ^= true;
		}

		return sum;
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

	double computeWeigthedIntensity(vi& elements, cv::Point point) {
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
	static bool getActualPixels(cv::Mat& image, vi& output, int yLimit) {
		vi result;
		cv::findNonZero(image, result);
		yLimit = abs(image.rows - yLimit);
		for (auto& p : result) {
			if (p.y <= yLimit)
				output.push_back(p);
		}
		return !output.empty();
	}

	static bool getActualPixels(vi& pixels, vi&target, double yLimit, int imageHeight) {
		if (!target.empty())
			target.clear();

		yLimit = abs(imageHeight - yLimit);

		for (auto& p : pixels) {
			if (p.y <= yLimit)
				target.push_back(p);
		}

		return !target.empty();
	}

	bool computeDiags(cv::Mat& image, vector<cv::Mat>& diags);

	bool computeDiagAvg(vector<cv::Mat>& diagonals, vd& output);

};
