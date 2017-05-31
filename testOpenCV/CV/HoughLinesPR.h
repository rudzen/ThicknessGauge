#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"
#include <opencv2/core/affine.hpp>
#include "Exceptions/NoLineDetectedException.h"
#include "HoughLinesR.h"

/*
	|  __
	| /__\
	| X~~|			"The eternal code god
	|-\|//-.		 watches over this mess."
   /|`.|'.' \			- R.A.Kohn, 2017
  |,|.\~~ /||
  |:||   ';||
  ||||   | ||
  \ \|     |`.
  |\X|     | |
  | .'     |||
  | |   .  |||
  |||   |  `.| JS
  ||||  |   ||
  ||||  |   ||
  `+.__._._+*/

using namespace tg;

class HoughLinesPR : public BaseR {

public:

	typedef struct LineH {
		cv::Vec4f entry;
		linePair points;
		std::vector<cv::Point2f> elements;

		LineH() {
		}

		LineH(cv::Vec4f entry, linePair points)
			: entry(entry),
			  points(points) {
			elements.reserve(cvRound(Util::dist_manhattan(points.first.x, points.second.x, points.first.y, points.second.y)));
		}

		friend bool operator==(const LineH& lhs, const LineH& rhs) {
			return lhs.entry == rhs.entry
				&& lhs.points == rhs.points
				&& lhs.elements == rhs.elements;
		}

		friend bool operator!=(const LineH& lhs, const LineH& rhs) {
			return !(lhs == rhs);
		}

		friend std::ostream& operator<<(std::ostream& os, const LineH& obj) {
			return os
				<< "entry: " << obj.entry
				<< " points(1/2): " << obj.points.first << '/' << obj.points.second
				<< " elements: " << obj.elements;
		}
	} LineH;

	struct lineHsizeSort {
		bool operator()(LineH l1, LineH l2) const { return l1.elements.size() < l2.elements.size(); }
	} lineHsizeSort;

	struct lineHYSort {
		bool operator()(cv::Point2f p1, cv::Point2f p2) const { return p1.y < p2.y; }
	} lineHYSort;

private:

	cv::Mat output;

	vector<cv::Vec4f> lines;

	vector<LineH> allLines;
	vector<LineH> rightLines;
	vector<LineH> leftLines;


public:
	const vector<LineH>& getAllLines() const {
		return allLines;
	}

	const vector<LineH>& getRightLines() const {
		return rightLines;
	}

	const vector<LineH>& getLeftLines() const {
		return leftLines;
	}

private:
	double center_;

	double leftY_ = 0.0;

	int rho_;

	int theta_;

	double angle_;

	int threshold_;

	int minLineLen_;

	int maxLineGab_;

	const int APER_MIN = 3;

	const int APER_MAX = 7;

	double minTheta_;

	double maxTheta_;

	int iAngleLimit_;

	double angleLimit_;

	bool showWindow_;

public:

	HoughLinesPR(const int rho, const int theta, const int threshold, const int minLineLen, const bool showWindow)
		: rho_(rho),
		  theta_(theta),
		  threshold_(threshold),
		  minLineLen_(minLineLen),
		  showWindow_(showWindow) {
		angle_ = degree * theta;
		minTheta_ = 0.0;
		maxTheta_ = CV_PI;
		angleLimit_ = 0;
		windowName = "HoughLinesP";
		maxLineGab_ = 10;
		if (showWindow)
			createWindow();
	}

private:
	void createWindow() {
		namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("rho", windowName, &rho_, 3, rhocb, this);
		cv::createTrackbar("theta", windowName, &theta_, 180, thetacb, this);
		cv::createTrackbar("threshold", windowName, &threshold_, 100, thresholdcb, this);
		cv::createTrackbar("min len", windowName, &minLineLen_, 200, minLineLencb, this);
		cv::createTrackbar("max gab", windowName, &maxLineGab_, 100, maxLineGabcb, this);
	}

	void computeBorders();

	void bresenham();

	static linePair HoughLinesPR::computePointPair(cv::Vec4f& line);

	double getAngle(cv::Vec4f& vec) const;

	double getAngle(cv::Point& p1, cv::Point& p2) const;

	double getAngle(int x1, int x2, int y1, int y2) const;

	static bool splitLinesInX(vector<LineH>& source, vector<LineH>& right, vector<LineH>& left, double x, double* leftCenter, double* rightCenter);

	// callbacks

	static void rhocb(int value, void* userData);

	static void thetacb(int value, void* userData);

	static void thresholdcb(int value, void* userData);

	static void minLineLencb(int value, void* userData);

	static void maxLineGabcb(int value, void* userData);

	void setRho(int rho) {
		this->rho_ = rho;
	}

	void setTheta(int theta) {
		if (theta == 0)
			theta++;
		this->theta_ = theta;
		angle_ = degree * theta;
	}

	void setThreshold(int threshold) {
		this->threshold_ = threshold;
	}

public:

	void doHorizontalHough();

	void drawLine(vector<linePair>& linePairs, cv::Scalar colour);

	void drawLines(vector<cv::Vec4f>& lines, cv::Scalar colour);

	void drawLines(vector<LineH>& lines, cv::Scalar colour);

	void drawLine(cv::Point2f& p1, cv::Point2f& p2, cv::Scalar colour);

	void drawLine(cv::Point& p1, cv::Point& p2, cv::Scalar colour);

	void drawLine(int x1, int y1, int x2, int y2, cv::Scalar colour);

	void drawLine(float x1, float y1, float x2, float y2, cv::Scalar colour);

	void drawLine(cv::Vec4f& line, cv::Scalar colour);

	void show() const;

	void setAngleLimit(double angleLimit) {
		this->angleLimit_ = angleLimit;
	}

	void setOriginal(cv::Mat& newImage) {
		original_ = newImage;
		if (showWindows_)
			cvtColor(original_, output, CV_GRAY2BGR);
	}

	const int& getMinLineLen() const {
		return minLineLen_;
	}

	void setMinLineLen(int minLineLen) {
		this->minLineLen_ = minLineLen;
	}

	const int& getMaxLineGab() const {
		return maxLineGab_;
	}

	void setMaxLineGab(int maxLineGab) {
		this->maxLineGab_ = maxLineGab;
	}
};

inline void HoughLinesPR::computeBorders() {


}

inline void HoughLinesPR::rhocb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setTheta(value);
	cout << cv::format("%s rho : %i\n", that->windowName, value);
}

inline void HoughLinesPR::thetacb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setTheta(value);
	cout << cv::format("%s theta : %i\n", that->windowName, value);
}

inline void HoughLinesPR::thresholdcb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setThreshold(value);
	cout << cv::format("%s threshold : %i\n", that->windowName, value);
}

inline void HoughLinesPR::maxLineGabcb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setMaxLineGab(value);
	cout << cv::format("%s maxLineGab : %i\n", that->windowName, value);
}

inline void HoughLinesPR::minLineLencb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setMinLineLen(value);
	cout << cv::format("%s minLineLen : %i\n", that->windowName, value);
}

inline void HoughLinesPR::doHorizontalHough() {
	// not optimized what so ever..
	// splitting things up in smaller function would help!
	lines.reserve(image_.cols * image_.rows);

	HoughLinesP(image_, lines, rho_, CV_PI / 4.0, threshold_, static_cast<double>(minLineLen_), static_cast<double>(maxLineGab_));

	auto count = lines.size();

	// set up data containers.

	allLines.clear();
	allLines.reserve(count);

	leftLines.clear();
	leftLines.reserve(count);

	rightLines.clear();
	rightLines.reserve(count);

	center_ = static_cast<double>(image_.cols) * 0.5f;

	// insert lines into data structure.
	for (auto& line : lines)
		allLines.emplace_back(LineH(line, computePointPair(line)));

	bresenham();

	//drawLines(allLines, cv::Scalar(0, 0, 255));

	if (showWindows_) {
		drawLines(leftLines, cv::Scalar(255, 0, 255));
		drawLines(rightLines, cv::Scalar(0, 255, 0));
		show();
	}


	return;

	// TODO : Re-make splitX to be "recursive" with splitting depth.

	double centerRight[3] = {0.0, 0.0, 0.0};
	double centerLeft[3] = {0.0, 0.0, 0.0};

	std::vector<LineH> left[3];
	std::vector<LineH> right[3];

	auto split = splitLinesInX(allLines, right[0], left[0], center_, &centerLeft[0], &centerRight[0]);

	//cout << "Center left  0 avg: " << centerLeft[0] << endl;
	//cout << "Center right 0 avg: " << centerRight[0] << endl;

	if (split) {
		if (centerRight[0] != 0.0)
			drawLines(right[0], cv::Scalar(255, 255, 0));

		if (centerLeft[0] != 0.0)
			drawLines(left[0], cv::Scalar(255, 0, 255));
	}

	// igen igen for HØJRE side

	split = splitLinesInX(left[0], right[1], left[1], centerLeft[0], &centerLeft[1], &centerRight[1]);

	//cout << "Center left  1 avg: " << centerLeft[1] << endl;
	//cout << "Center right 1 avg: " << centerRight[1] << endl;

	if (split) {
		if (centerRight[1] != 0.0)
			drawLines(right[1], cv::Scalar(0, 0, 0));

		if (centerLeft[1] != 0.0)
			drawLines(left[1], cv::Scalar(255, 255, 255));
	}

	// for helvede.. igen for venstre

	split = splitLinesInX(right[0], right[2], left[2], centerRight[0], &centerLeft[2], &centerRight[2]);

	//cout << "Center left  2 avg: " << centerLeft[2] << endl;
	//cout << "Center right 2 avg: " << centerRight[2] << endl;

	if (split) {
		if (centerRight[2] != 0.0)
			drawLines(right[2], cv::Scalar(255, 255, 255));

		if (centerLeft[2] != 0.0)
			drawLines(left[2], cv::Scalar(0, 0, 0));
	}

	if (right[1].empty())
		return;

	// find lige den mest avg
	double sumY = 0.0;
	double sumX = 0.0;

	auto maxX = right[1].back().entry[2];
	auto minX = right[1].front().entry[0];
	auto maxY = right[1].back().entry[1];
	auto minY = right[1].front().entry[1];

	for (auto& v : right[1]) {

		// check furthest point in X for max
		if (v.entry[2] > maxX)
			maxX = v.entry[2];

		// check closest point to 0 for min
		if (v.entry[0] < minX)
			minX = v.entry[0];

		sumX += v.entry[0] + v.entry[2];
		sumY += v.entry[1] + v.entry[3];
	}

	if (sumX != 0.0)
		sumX /= right[1].size() * 0.5f;

	if (sumY != 0.0)
		sumY /= right[1].size() * 0.5f;

	std::cout << "Right side Y sum : " << sumY << std::endl;

	drawLine(0, cvRound(sumY), cvRound(maxX), cvRound(sumY), cv::Scalar(0, 0, 0));

	//cout << "SumX: " << sumX << " SumY: " << sumY << " maxX: " << maxX << " maxY: " << maxY << endl;

	show();
}

/**
 * \brief Populates the lines information for main vector and populates left and right sides
 */
inline void HoughLinesPR::bresenham() {

	if (allLines.empty())
		return;

	auto size = allLines.size();

	rightLines.clear();
	rightLines.reserve(size);

	leftLines.clear();
	leftLines.reserve(size);

	for (auto& line : allLines) {
		if (line.entry[0] < center_)
			leftLines.emplace_back(line);
		else
			rightLines.emplace_back(line);

		//if (line.entry[0] + ((line.entry[2] - line.entry[0]) / 2) < center_)
		//	leftLines.emplace_back(line);
		//else
		//	rightLines.emplace_back(line);
	}

	auto lSize = leftLines.size();
	auto rSize = rightLines.size();

	auto onlyRight = false;

	//if (rSize == 0) {
	//	if (lSize == 0)
	//		throw NoLineDetectedException("No horizontal lines detected.");

	//	// emergency case if the only located lines are on the left side.
	//	Util::copyVector(leftLines, rightLines);
	//	leftLines.clear();
	//	Util::loge("Warning, no right side lines were detected, left lines treated as right lines.");
	//	onlyRight ^= true;
	//}
	//else
	//	onlyRight = lSize == 0;


	// build right side line points
	for (auto& rightLine : rightLines) {
		cv::LineIterator it(image_, rightLine.points.first, rightLine.points.second, 8);
		rightLine.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++ , ++it)
			rightLine.elements.emplace_back(it.pos());
	}

	//// sort if needed
	//if (rSize > 1)
	//	sort(rightLines.begin(), rightLines.end(), lineHsizeSort);

	//// null left side lines guard
	//if (onlyRight)
	//	return;

	// build left side line points
	for (auto& leftLine : leftLines) {
		cv::LineIterator it(image_, leftLine.points.first, leftLine.points.second, 8);
		leftLine.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++ , ++it)
			leftLine.elements.emplace_back(it.pos());
	}

	//// sort if needed
	//if (lSize > 1)
	//	sort(leftLines.begin(), leftLines.end(), lineHsizeSort);

}

inline linePair HoughLinesPR::computePointPair(cv::Vec4f& line) {
	return linePair(cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]));
}

inline double HoughLinesPR::getAngle(cv::Vec4f& vec) const {
	return atan2(vec[1] - vec[3], vec[0] - vec[2]);
}

inline double HoughLinesPR::getAngle(cv::Point& p1, cv::Point& p2) const {
	return atan2(p1.y - p2.y, p1.x - p2.x);
}

inline double HoughLinesPR::getAngle(int x1, int x2, int y1, int y2) const {
	return atan2(y1 - y2, x1 - x2);
}

inline bool HoughLinesPR::splitLinesInX(vector<LineH>& source, vector<LineH>& right, vector<LineH>& left, double x, double* leftCenter, double* rightCenter) {

	*leftCenter = 0.0;
	*rightCenter = 0.0;

	for (auto& line : source) {
		//if (s[1] >= yMin && s[3] >= yMin) { // desværre, ellers bliver størrelserne og dermed pointers fucked up.
		auto centerX = (line.entry[2] + line.entry[0]) * 0.5f;
		if (centerX <= x) {
			left.emplace_back(line);
			*leftCenter += centerX;
		}
		else {
			right.emplace_back(line);
			*rightCenter += centerX;
		}
		//}
	}

	if (!left.empty())
		*leftCenter /= left.size();

	if (!right.empty())
		*rightCenter /= right.size();

	return !(right.empty() && left.empty());

}

inline void HoughLinesPR::drawLine(vector<linePair>& linePairs, cv::Scalar colour) {
	if (!showWindow_)
		return;

	for (auto& r : linePairs) {
		drawLine(r.first, r.second, colour);
		//line(original, r.first, r.second, colour, 1, CV_AA);
	}
}

inline void HoughLinesPR::drawLines(vector<cv::Vec4f>& lines, cv::Scalar colour) {
	if (!showWindow_)
		return;

	for (auto& line : lines)
		drawLine(line, colour);
}

inline void HoughLinesPR::drawLines(vector<LineH>& lines, cv::Scalar colour) {
	if (!showWindow_)
		return;

	for (auto& line : lines)
		drawLine(line.entry, colour);
}

inline void HoughLinesPR::drawLine(cv::Point2f& p1, cv::Point2f& p2, cv::Scalar colour) {
	line(output, p1, p2, colour, 1, CV_AA);
}

inline void HoughLinesPR::drawLine(cv::Point& p1, cv::Point& p2, cv::Scalar colour) {
	line(output, p1, p2, colour, 1, CV_AA);
}

inline void HoughLinesPR::drawLine(cv::Vec4f& line, cv::Scalar colour) {
	drawLine(line[0], line[1], line[2], line[3], colour);
}

inline void HoughLinesPR::drawLine(int x1, int y1, int x2, int y2, cv::Scalar colour) {
	cv::Point p1(x1, y1);
	cv::Point p2(x2, y2);
	drawLine(p1, p2, colour);
}

inline void HoughLinesPR::drawLine(float x1, float y1, float x2, float y2, cv::Scalar colour) {
	cv::Point p1(Util::round(x1), Util::round(y1));
	cv::Point p2(Util::round(x2), Util::round(y2));
	drawLine(p1, p2, colour);
}

inline void HoughLinesPR::show() const {
	imshow(windowName, output);
}
