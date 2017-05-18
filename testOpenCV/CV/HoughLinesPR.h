#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"
#include "Pixel.h"
#include <opencv2/core/affine.hpp>
#include "Exceptions/NoLineDetectedException.h"

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

class HoughLinesPR : public BaseR<float> {

public:

	typedef struct LineH {
		cv::Vec4f entry;
		linePair points;
		std::vector<cv::Point2f> elements;

		LineH() { }

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
	} LineV;

	struct lineHsizeSort {
		bool operator()(LineV l1, LineV l2) const { return l1.elements.size() < l2.elements.size(); }
	} lineHsizeSort;

	struct lineHYSort {
		bool operator()(cv::Point2f p1, cv::Point2f p2) const { return p1.y < p2.y; }
	} lineHYSort;

private:

	cv::Mat output;

	vector<cv::Vec4f> lines;

	vector<LineV> allLines;
	vector<LineV> rightLines;
	vector<LineV> leftLines;


public:
	const vector<LineV>& getAllLines() const {
		return allLines;
	}

	const vector<LineV>& getRightLines() const {
		return rightLines;
	}

	const vector<LineV>& getLeftLines() const {
		return leftLines;
	}

private:
	float center;

	double leftY = 0.0;

	int rho;

	int theta;

	double angle;

	int threshold;

	int minLineLen;

	int maxLineGab;

	const int aperMin = 3;

	const int aperMax = 7;

	double minTheta;

	double maxTheta;

	int iAngleLimit;

	double angleLimit;

	bool showWindow;

public:

	HoughLinesPR(const int rho, const int theta, const int threshold, const int minLineLen, const bool showWindow)
		: rho(rho),
		theta(theta),
		threshold(threshold),
		minLineLen(minLineLen),
		showWindow(showWindow) {
		angle = degree * theta;
		minTheta = 0.0;
		maxTheta = CV_PI;
		angleLimit = 0;
		windowName = "HoughLinesP";
		maxLineGab = 10;
		if (showWindow)
			createWindow();
	}

private:
	void createWindow() {
		namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("rho", windowName, &rho, 3, rhocb, this);
		cv::createTrackbar("theta", windowName, &theta, 180, thetacb, this);
		cv::createTrackbar("threshold", windowName, &threshold, 100, thresholdcb, this);
		cv::createTrackbar("min len", windowName, &minLineLen, 200, minLineLencb, this);
		cv::createTrackbar("max gab", windowName, &maxLineGab, 100, maxLineGabcb, this);
	}

	void computeBorders();

	void bresenham();

	static linePair HoughLinesPR::computeLinePair(cv::Vec4f& line);

	double getAngle(cv::Vec4f& vec) const;

	double getAngle(cv::Point& p1, cv::Point& p2) const;

	double getAngle(int x1, int x2, int y1, int y2) const;

	static bool splitX(vector<LineH>& source, vector<LineH>& right, vector<LineH>& left, double x, double *leftCenter, double *rightCenter);

	// callbacks

	static void rhocb(int value, void* userData);

	static void thetacb(int value, void* userData);

	static void thresholdcb(int value, void* userData);

	static void minLineLencb(int value, void* userData);

	static void maxLineGabcb(int value, void* userData);

	void setRho(int rho) {
		this->rho = rho;
	}

	void setTheta(int theta) {
		if (theta == 0)
			theta++;
		this->theta = theta;
		angle = degree * theta;
	}

	void setThreshold(int threshold) {
		this->threshold = threshold;
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

	void drawLine(cv::Vec4f & line, cv::Scalar colour);

	void show() const;

	void alignLeftY(int frameCount) {
		leftY /= frameCount;
		cout << "Horizontal baseline aligned to : " << leftY << " y" << endl;
		line(output, cv::Point(0, cvRound(leftY)), cv::Point(output.cols / 2, cvRound(leftY)), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		imshow(windowName, output);
	}

	void setAngleLimit(double angleLimit) {
		this->angleLimit = angleLimit;
	}

	void setOriginal(cv::Mat& newImage) {
		original_ = newImage;
		cvtColor(original_, output, CV_GRAY2BGR);
		lines.reserve(image_.cols * image_.rows);
	}

	const int& getMinLineLen() const {
		return minLineLen;
	}

	void setMinLineLen(int minLineLen) {
		this->minLineLen = minLineLen;
	}

	const int& getMaxLineGab() const {
		return maxLineGab;
	}

	void setMaxLineGab(int maxLineGab) {
		this->maxLineGab = maxLineGab;
	}
};

inline void HoughLinesPR::computeBorders() {



}

inline void HoughLinesPR::rhocb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setTheta(value);
	cout << "Hough rho : " << value << endl;
}

inline void HoughLinesPR::thetacb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setTheta(value);
	cout << "Hough theta : " << value << endl;
}

inline void HoughLinesPR::thresholdcb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setThreshold(value);
	cout << "Hough threshold : " << value << endl;
}

inline void HoughLinesPR::maxLineGabcb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setMaxLineGab(value);
	cout << "maxLineGabcb : " << value << endl;
}

inline void HoughLinesPR::minLineLencb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setMinLineLen(value);
	cout << "minLineLencb : " << value << endl;
}

inline void HoughLinesPR::doHorizontalHough() {
	// not optimized what so ever..
	// splitting things up in smaller function would help!

	HoughLinesP(image_, lines, rho, CV_PI / 4, threshold, static_cast<double>(minLineLen), static_cast<double>(maxLineGab));

	auto count = lines.size();

	// set up data containers.

	allLines.clear();
	allLines.reserve(count);

	leftLines.clear();
	leftLines.reserve(count);

	rightLines.clear();
	rightLines.reserve(count);

	center = image_.cols / 2;

	// insert lines into data structure.
	for (auto& l : lines)
		allLines.push_back(LineH(l, computeLinePair(l)));

	bresenham();

	//drawLines(allLines, cv::Scalar(0, 0, 255));

	drawLines(leftLines, cv::Scalar(255, 0, 255));
	drawLines(rightLines, cv::Scalar(0, 0, 0));

	show();

	return;
	// TODO : Re-make splitX to be "recursive" with splitting depth.

	double centerRight[3] = { 0.0, 0.0, 0.0 };
	double centerLeft[3] = { 0.0, 0.0, 0.0 };

	vector<LineH> left[3];
	vector<LineH> right[3];

	auto split = splitX(allLines, right[0], left[0], center, &centerLeft[0], &centerRight[0]);

	//cout << "Center left  0 avg: " << centerLeft[0] << endl;
	//cout << "Center right 0 avg: " << centerRight[0] << endl;

	if (split) {
		if (centerRight[0] != 0.0)
			drawLines(right[0], cv::Scalar(255, 255, 0));

		if (centerLeft[0] != 0.0)
			drawLines(left[0], cv::Scalar(255, 0, 255));
	}

	// igen igen for HØJRE side

	split = splitX(left[0], right[1], left[1], centerLeft[0], &centerLeft[1], &centerRight[1]);

	//cout << "Center left  1 avg: " << centerLeft[1] << endl;
	//cout << "Center right 1 avg: " << centerRight[1] << endl;

	if (split) {
		if (centerRight[1] != 0.0)
			drawLines(right[1], cv::Scalar(0, 0, 0));

		if (centerLeft[1] != 0.0)
			drawLines(left[1], cv::Scalar(255, 255, 255));
	}

	// for helvede.. igen for venstre

	split = splitX(right[0], right[2], left[2], centerRight[0], &centerLeft[2], &centerRight[2]);

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

	cout << "Right side Y sum : " << sumY << endl;

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

	for (auto& a : allLines) {
		if (a.entry[0] + ((a.entry[2] - a.entry[0]) / 2) < center)
			leftLines.push_back(a);
		else
			rightLines.push_back(a);
	}

	auto lSize = leftLines.size();
	auto rSize = rightLines.size();

	auto onlyRight = false;

	if (rSize == 0) {
		if (lSize == 0)
			throw NoLineDetectedException("No horizontal lines detected.");

		// emergency case if the only located lines are on the left side.
		Util::copyVector(leftLines, rightLines);
		leftLines.clear();
		Util::loge("Warning, no right side lines were detected, left lines treated as right lines.");
		onlyRight ^= true;
	} else
		onlyRight = lSize == 0;


	// build right side line points
	for (auto& right : rightLines) {
		cv::LineIterator it(image_, right.points.first, right.points.second, 8);
		right.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++, ++it)
			right.elements.push_back(it.pos());
	}

	// sort if needed
	if (rSize > 1)
		sort(rightLines.begin(), rightLines.end(), lineHsizeSort);

	// null left side lines guard
	if (onlyRight)
		return;

	// build left side line points
	for (auto& left : leftLines) {
		cv::LineIterator it(image_, left.points.first, left.points.second, 8);
		left.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++, ++it)
			left.elements.push_back(it.pos());
	}

	// sort if needed
	if (lSize > 1)
		sort(leftLines.begin(), leftLines.end(), lineHsizeSort);

}

inline linePair HoughLinesPR::computeLinePair(cv::Vec4f& line) {
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

inline bool HoughLinesPR::splitX(vector<LineH>& source, vector<LineH>& right, vector<LineH>& left, double x, double *leftCenter, double *rightCenter) {

	*leftCenter = 0.0;
	*rightCenter = 0.0;

	for (auto& s : source) {
		//if (s[1] >= yMin && s[3] >= yMin) { // desværre, ellers bliver størrelserne og dermed pointers fucked up.
			auto centerX = (s.entry[2] + s.entry[0]) * 0.5f;
			if (centerX <= x) {
				left.push_back(s);
				*leftCenter += centerX;
			} else {
				right.push_back(s);
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
	if (!showWindow)
		return;

	for (auto& r : linePairs) {
		drawLine(r.first, r.second, colour);
		//line(original, r.first, r.second, colour, 1, CV_AA);
	}
}

inline void HoughLinesPR::drawLines(vector<cv::Vec4f>& lines, cv::Scalar colour) {
	if (!showWindow)
		return;

	for (auto& l : lines)
		drawLine(l, colour);
}

inline void HoughLinesPR::drawLines(vector<LineH>& lines, cv::Scalar colour) {
	if (!showWindow)
		return;

	for (auto& l : lines)
		drawLine(l.entry, colour);
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
