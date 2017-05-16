#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"

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

using namespace _cv;

class HoughLinesR : public BaseR {

public:

	enum class Type {
		Regular, Properlistic
	};

private:

	cv::Mat output;

	vector<cv::Vec2f> lines;

	vector<cv::Vec4f> linesP;

	vector<linePair> linePointsVert;

	vector<linePair> linePointsHori;

	vector<cv::Point2i> rightBorder;

	vector<cv::Point2i> leftBorder;

	double leftY = 0.0;

	//const std::string windowName = "HoughLines";

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

	Type properlistic;

public:

	HoughLinesR(const int rho, const int theta, const int threshold, const bool showWindow, const Type properlistic)
		: rho(rho),
		  theta(theta),
		  threshold(threshold),
		  showWindow(showWindow),
		  properlistic(properlistic)
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

private:
	void createWindow() {
		namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("rho", windowName, &rho, 3, rhocb, this);
		cv::createTrackbar("theta", windowName, &theta, 180, thetacb, this);
		cv::createTrackbar("threshold", windowName, &threshold, 100, thresholdcb, this);
	}

	void computeBorders();
	
	// callbacks

	static void rhocb(int value, void* userData);

	static void thetacb(int value, void* userData);

	static void thresholdcb(int value, void* userData);

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

	void doHorizontalHough();

	linePair HoughLinesR::computeLinePair(cv::Vec2f& line) const;

	void drawLines(vector<linePair>& linePairs, cv::Scalar colour);

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

	const vector<cv::Vec4f>& getLinesP() const {
		return linesP;
	}
};

inline void HoughLinesR::computeBorders() {

	if (linePointsVert.empty())
		return;

	auto delta = 3;

	auto midX = output.cols / 2;
	auto midY = output.rows / 2;

	vector<cv::Point2i> left;
	vector<cv::Point2i> right;

	const auto linePointsSize = linePointsVert.size();

	// reserve enough space

	//left.reserve(linePointsSize);
	//right.reserve(linePointsSize);

	//for (auto& lp : linePointsVert) {
	//	cv::LineIterator it(output, lp.first, lp.second, 8);
	//	for (auto i = 0; i < it.count; i++, ++it) {
	//		auto pt = it.pos();

	//		// check could possibly be moved outside this loop
	//		// but since there is no guarentee for anything, leave it for now
	//		if (pt.x < midX)
	//			left.push_back(pt);
	//		else
	//			right.push_back(pt);
	//	}
	//}

	//std::cout << "left vert lines : " << left.size() << endl;
	//std::cout << "right vert lines : " << right.size() << endl;

	// test borders

	midX = output.rows / 2;

	cv::Point2i bestTop(output.cols, output.rows);
	cv::Point2i bestLow(output.cols, output.rows);

	auto best = 0.0;

	for (auto& p : linePointsVert) {

		auto dist = cv::norm(p.first - p.second);
		if (dist > best) {
			best = dist;
			if (p.first.y < p.second.y) {
				bestTop = p.first;
				bestLow = p.second;
			} else {
				bestTop = p.second;
				bestLow = p.first;
			}
		}
	}


	line(output, bestLow, bestTop, cv::Scalar(0, 255, 255), 3);

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

	linePointsVert.clear();
	linePointsVert.reserve(lines.size());

	for (auto& l : lines) {
		auto theta = l[1];
		if (theta <= degree * (180 - angleLimit) && theta >= degree * angleLimit)
			continue;
		auto p = computeLinePair(l);
		linePointsVert.push_back(p);
		//std::cout << "vert : " << p.first << " " << p.second << endl;
	}

	//std::cout << ' ' << linePointsVert.size() << " fits..\n";

	drawLines(linePointsVert, cv::Scalar(255, 0, 255));
	computeBorders();
}

inline void HoughLinesR::doHorizontalHough() {

	if (!lines.empty())
		lines.clear();

	HoughLines(image_, lines, 1, degree, threshold, 0, 0);
	//cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);

	//std::cout << "doHorizontalHough # lines : " << lines.size();

	if (lines.empty())
		return;

	linePointsHori.clear();
	linePointsHori.reserve(lines.size());

	for (auto& l : lines) {
		auto theta = l[1];
		if (theta <= degree * 89.9 || theta >= degree * 90.1)
			continue;
		auto p = computeLinePair(l);
		if (p.first.y > original_.rows / 3)
			linePointsHori.push_back(p);		
	}

	//std::cout << ' ' << linePointsHori.size() << " fits..\n";

	// quick test
	auto sum = 0.0;
	for (auto& l : linePointsHori) {
		sum += l.first.y;
	}

	sum /= linePointsHori.size();

	leftY += sum;

	drawLines(linePointsHori, cv::Scalar(255, 0, 0));
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

inline void HoughLinesR::drawLines(vector<linePair>& linePairs, cv::Scalar colour) {
	if (!showWindow)
		return;

	for (auto& r : linePairs) {
		line(output, r.first, r.second, colour, 1, CV_AA);
		//line(original, r.first, r.second, colour, 1, CV_AA);
	}
	imshow(windowName, output);
}
