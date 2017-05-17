#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"
#include "../tg.h"

/*
(      -4QQQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
(        4QQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
( )QQQm. ]QQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
( )WQQQ; ]Qf =QQQ  dQQ@^ -4: jQ(      QQ
( )WQQD  jQf =QQQ  dQW`  .   jQc___   QQ
(       jWQf =QQQ  dQf .mQc  jQQQQF  jQQ
(       ?WQf =QQQ  dQ; ]QQQ  jQQQP  jWQQ
( )WQQL  WWf =QQQ  dQ: jQQQ. jQQD  <QWQQ
( )WQQW  dQf :QQW  dQ; )QQ@  jQ@` _QQQQQ
( )WQQm  3Qk  ??'  dQL  "T'  jQ'  TTTTQQ
( )WQQQ  3QQ,   <  dQQ,   _. jQ       WW
wawWQQQwaaQQQwawWaamQQmc_wmwayQaaaaaaamQ
QWWQQQQWWQQQQQWQQQWQQQQQQQQWWQQQWQWQWQQQ
*/

using namespace tg;
using namespace std;

class HoughLinesR : public BaseR {

public:

	typedef struct LineV {
		cv::Vec2f entry;
		linePair points;
		std::vector<cv::Point2f> elements;

		LineV() { }

		LineV(cv::Vec2f entry, linePair points)
			: entry(entry),
			points(points) {
			elements.reserve(cvRound(Util::dist_manhattan(points.first.x, points.second.x, points.first.y, points.second.y)));
		}

		friend bool operator==(const LineV& lhs, const LineV& rhs) {
			return lhs.entry == rhs.entry
				&& lhs.points == rhs.points
				&& lhs.elements == rhs.elements;
		}

		friend bool operator!=(const LineV& lhs, const LineV& rhs) {
			return !(lhs == rhs);
		}

		friend std::ostream& operator<<(std::ostream& os, const LineV& obj) {
			return os
				<< "entry: " << obj.entry
				<< " points(1/2): " << obj.points.first << '/' << obj.points.second
				<< " elements: " << obj.elements;
		}
	} LineV;

	struct lineVsizeSort {
		bool operator()(LineV l1, LineV l2) const { return l1.elements.size() < l2.elements.size(); }
	} lineVsizeSort;

	struct lineVYSort {
		bool operator()(cv::Point2f p1, cv::Point2f p2) const { return p1.y < p2.y; }
	} lineVYSort;

private:

	cv::Mat output;

	vector<cv::Vec2f> lines;

	vector<LineV> allLines;
	vector<LineV> rightLines;
	vector<LineV> leftLines;

	LineV rightBorder;
	LineV leftBorder;

	double leftY = 0.0;

	int rho;

	int theta;

	double angle;

	int threshold;

	const int aperMin = 3;
	const int aperMax = 7;

	double srn;

	double stn;

	double minTheta;

	double maxTheta;

	int iAngleLimit;

	double angleLimit;

	bool showWindow;

public:

	HoughLinesR(const int rho, const int theta, const int threshold, const bool showWindow)
		: rho(rho),
		  theta(theta),
		  threshold(threshold),
		  showWindow(showWindow)
	{
		angle = CV_PI / 180 * theta;
		srn = 0;
		stn = 0;
		minTheta = 0.0;
		maxTheta = CV_PI;
		angleLimit = 0;
		windowName = "HoughLines";
		if (showWindow)
			createWindow();
	}

	void computeBorders();

private:
	void createWindow() {
		namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("rho", windowName, &rho, 3, rhocb, this);
		cv::createTrackbar("theta", windowName, &theta, 180, thetacb, this);
		cv::createTrackbar("threshold", windowName, &threshold, 100, thresholdcb, this);
	}

	linePair HoughLinesR::computeLinePair(cv::Vec2f& line) const;

	void drawLines(vector<LineV>& linePairs, cv::Scalar colour);

	void showOutput() const;

	void bresenham();

	static void computeRectFromLines(vector<LineV>& input, cv::Rect2f& output);

	// callbacks

	static void rhocb(int value, void* userData);

	static void thetacb(int value, void* userData);

	static void thresholdcb(int value, void* userData);

	// getters & setters

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

	void doVerticalHough();

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
	}

	const vector<cv::Vec2f>& getLines() const {
		return lines;
	}

	const vector<LineV>& getAllLines() const {
		return allLines;
	}

	void setAllLines(const vector<LineV>& allLines) {
		this->allLines = allLines;
	}

	const vector<LineV>& getRightLines() const {
		return rightLines;
	}

	void setRightLines(const vector<LineV>& rightLines) {
		this->rightLines = rightLines;
	}

	const vector<LineV>& getLeftLines() const {
		return leftLines;
	}

	void setLeftLines(const vector<LineV>& leftLines) {
		this->leftLines = leftLines;
	}
};

inline void HoughLinesR::computeRectFromLines(vector<LineV>& input, cv::Rect2f& output) {
	
	for (auto& l : input) {
		cv::Rect2f t = cv::boundingRect(l.elements);
		output.x += t.x;
		output.y += t.y;
		output.width += t.width;
	}

	const auto size = input.size();

	output.x /= size;
	output.y /= size;
	output.width /= size;

}

inline void HoughLinesR::computeBorders() {

	cv::Rect2f leftRoi(0.0f, 0.0f, 0.0f, static_cast<float>(image_.rows));
	cv::Rect2f rightRoi(0.0f, 0.0f, 0.0f, static_cast<float>(image_.rows));

	computeRectFromLines(leftLines, leftRoi);
	computeRectFromLines(rightLines, rightRoi);

	markingRect.x = leftRoi.x;
	markingRect.y = leftRoi.y;
	markingRect.width = rightRoi.x - leftRoi.x + rightRoi.width;
	markingRect.height = static_cast<float>(image_.rows);

	if (showWindow) {
		drawLines(leftLines, cv::Scalar(0, 255, 0));
		drawLines(rightLines, cv::Scalar(0, 0, 255));
		cv::rectangle(output, leftRoi, cv::Scalar(255, 255, 0), 3, CV_AA);
		cv::rectangle(output, rightRoi, cv::Scalar(255, 255, 0), 3, CV_AA);
		cv::rectangle(output, markingRect, cv::Scalar(255, 0, 0), 3, CV_AA);
		showOutput();
	}

	return;






	leftBorder.points.first.x = 0.0f;
	leftBorder.points.first.y = static_cast<float>(output.rows);

	leftBorder.points.second.x = 0.0f;
	leftBorder.points.second.y = 0.0f;

	for (auto& l : leftLines) {
		cout << "F: " << l.points.first << endl;
		cout << "S: " << l.points.second << endl;
		leftBorder.points.first.x += l.points.first.y;
		leftBorder.points.second.x += l.points.second.x;
	}

	leftBorder.points.first.x /= leftLines.size();
	leftBorder.points.second.x /= leftLines.size();

	//cout << "L avg: " << leftBorder.points.first << " / " << leftBorder.points.second << endl;

	if (showWindow) {
		drawLines(leftLines, cv::Scalar(0, 255, 0));
		drawLines(rightLines, cv::Scalar(0, 0, 255));
		line(output, leftBorder.points.first, leftBorder.points.second, cv::Scalar(255, 255, 0), 3, CV_AA);
		showOutput();
	}


}

inline void HoughLinesR::rhocb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setTheta(value);
	cout << "Hough rho : " << value << endl;
}

inline void HoughLinesR::thetacb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setTheta(value);
	cout << "Hough theta : " << value << endl;
}

inline void HoughLinesR::thresholdcb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setThreshold(value);
	cout << "Hough threshold : " << value << endl;
}

inline void HoughLinesR::doVerticalHough() {

	if (!lines.empty())
		lines.clear();

	//cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);
	HoughLines(image_, lines, 1, degree, threshold, 0, 0);

	//std::cout << "doVerticalHough # lines : " << lines.size();

	if (lines.empty())
		return;

	allLines.clear();
	allLines.reserve(lines.size());

	for (auto& l : lines) {
		auto theta = l[1];
		if (theta <= degree * (180 - angleLimit) && theta >= degree * angleLimit)
			continue;
		auto p = computeLinePair(l);
		allLines.push_back(LineV(l, p));
		//std::cout << "vert : " << p.first << " " << p.second << endl;
	}

	if (allLines.empty())
		cerr << "FATAL ERROR, NO VERTICAL LINES DETECTED!";

	//drawLines(allLines, cv::Scalar(255, 0, 255));
	bresenham();
}

inline void HoughLinesR::bresenham() {
	
	if (allLines.empty())
		return;

	auto size = allLines.size();

	rightLines.clear();
	rightLines.reserve(size);

	leftLines.clear();
	leftLines.reserve(size);

	auto center = image_.cols / 2;

	for (auto& a : allLines) {
		if (a.points.first.x < center)
			leftLines.push_back(a);
		else
			rightLines.push_back(a);
	}

	auto lSize = leftLines.size();
	auto rSize = rightLines.size();

	if (lSize == 0)
		cerr << "FATAL, no left lines detected.." << endl;

	for (auto& left : leftLines) {
		cv::LineIterator it(image_, left.points.first, left.points.second, 8);
		left.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++, ++it)
			left.elements.push_back(it.pos());
	}

	if (leftLines.size() > 1) {
		sort(leftLines.begin(), leftLines.end(), lineVsizeSort);
	}

	//for (auto& l : leftLines)
	//	cout << "L: " << l << endl;

	for (auto& right : rightLines) {
		cv::LineIterator it(image_, right.points.first, right.points.second, 8);
		right.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++, ++it)
			right.elements.push_back(it.pos());
	}

	if (rightLines.size() > 1) {
		sort(rightLines.begin(), rightLines.end(), lineVsizeSort);
	}

	//for (auto& l : rightLines)
	//	cout << "R: " << l << endl;


}

inline linePair HoughLinesR::computeLinePair(cv::Vec2f& line) const {
	auto rho = line[0];
	auto theta = line[1];
	double a = cos(theta);
	double b = sin(theta);
	auto x0 = a * rho;
	auto y0 = b * rho;
	cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
	cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
	return linePair(pt1, pt2);
}

inline void HoughLinesR::drawLines(vector<LineV>& linePairs, cv::Scalar colour) {
	if (!showWindow)
		return;

	if (linePairs.empty())
		return;

	for (auto& r : linePairs) {
		line(output, r.points.first, r.points.second, colour, 1, CV_AA);
		//line(original, r.first, r.second, colour, 1, CV_AA);
	}
}

inline void HoughLinesR::showOutput() const {
	if (!showWindow)
		return;

	imshow(windowName, output);
}
