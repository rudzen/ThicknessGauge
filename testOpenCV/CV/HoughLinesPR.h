#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"
#include "Pixel.h"
#include "HoughLinesR.h"

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

class HoughLinesPR : public BaseR {

public:

	enum class Type {
		Regular, Properlistic
	};

private:

	cv::Mat output;

	std::vector<cv::Vec4f> lines;

	std::vector<linePair> linePointsVert;

	std::vector<linePair> linePointsHori;

	std::vector<cv::Point2i> rightBorder;

	std::vector<cv::Point2i> leftBorder;

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

	HoughLinesPR(const int rho, const int theta, const int threshold, const bool showWindow)
		: rho(rho),
		theta(theta),
		threshold(threshold),
		showWindow(showWindow) {
		angle = degree * theta;
		minTheta = 0.0;
		maxTheta = CV_PI;
		angleLimit = 0;
		windowName = "HoughLinesP";
		minLineLen = 50;
		maxLineGab = 10;
		if (showWindow)
			createWindow();
	}

private:
	void createWindow() {
		cv::namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("rho", windowName, &rho, 3, rhocb, this);
		cv::createTrackbar("theta", windowName, &theta, 180, thetacb, this);
		cv::createTrackbar("threshold", windowName, &threshold, 100, thresholdcb, this);
		cv::createTrackbar("min len", windowName, &minLineLen, 200, minLineLencb, this);
		cv::createTrackbar("max gab", windowName, &maxLineGab, 100, maxLineGabcb, this);
	}

	void computeBorders();

	linePair HoughLinesPR::computeLinePair(cv::Vec2f& line) const;

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

	void doVerticalHough();

	void doHorizontalHough();

	void doHough(bool horizontal);

	void drawLines(std::vector<linePair>& linePairs, cv::Scalar colour);

	void show() const;

	void alignLeftY(int frameCount) {
		leftY /= frameCount;
		std::cout << "Horizontal baseline aligned to : " << leftY << " y" << endl;
		cv::line(output, cv::Point(0, cvRound(leftY)), cv::Point(output.cols / 2, cvRound(leftY)), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		cv::imshow(windowName, output);
	}

	void setAngleLimit(double angleLimit) {
		this->angleLimit = angleLimit;
	}

	void setOriginal(cv::Mat& newImage) {
		original_ = newImage;
		cv::cvtColor(original_, output, CV_GRAY2BGR);
	}

	const std::vector<cv::Vec4f>& getLines() const {
		return lines;
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

	if (linePointsVert.empty())
		return;

	auto delta = 3;

	auto midX = output.cols / 2;
	auto midY = output.rows / 2;

	std::vector<cv::Point2i> left;
	std::vector<cv::Point2i> right;

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

	double best = 0.0;

	for (auto& p : linePointsVert) {

		double dist = cv::norm(p.first - p.second);
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


	cv::line(output, bestLow, bestTop, cv::Scalar(0, 255, 255), 3);

}

inline void HoughLinesPR::rhocb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setTheta(value);
	std::cout << "Hough rho : " << value << std::endl;
}

inline void HoughLinesPR::thetacb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setTheta(value);
	std::cout << "Hough theta : " << value << std::endl;
}

inline void HoughLinesPR::thresholdcb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setThreshold(value);
	std::cout << "Hough threshold : " << value << std::endl;
}

inline void HoughLinesPR::maxLineGabcb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setMaxLineGab(value);
	std::cout << "maxLineGabcb : " << value << std::endl;
}

inline void HoughLinesPR::minLineLencb(int value, void* userData) {
	auto that = static_cast<HoughLinesPR*>(userData);
	that->setMinLineLen(value);
	std::cout << "minLineLencb : " << value << std::endl;
}

inline void HoughLinesPR::doHough(bool horizontal) {
	
	if (!lines.empty())
		lines.clear();

	lines.reserve(image_.cols * image_.rows);

	std::cout << "b4 hlines" << std::endl;

	//HoughLinesP(image_, lines, 1, CV_PI / 180, 50, 50, 10);
	cv::HoughLinesP(image_, lines, rho, degree, threshold, static_cast<double>(minLineLen), static_cast<double>(maxLineGab));

	std::cout << "after hlines" << std::endl;

	auto count = lines.size();

	std::cout << "HoughLinesP line count = " << count << std::endl;

	linePointsVert.clear();
	linePointsVert.reserve(count);

	linePointsHori.clear();
	linePointsHori.reserve(count);

	cv::Point center(image_.cols / 2, image_.rows / 2);

	std::vector<linePair> linLeft;
	std::vector<linePair> linRight;

	linLeft.reserve(count);
	linRight.reserve(count);

	for (auto& line : lines) {
		if (line[0] <= center.x && line[2] <= center.x)
			linLeft.push_back(linePair(cv::Point(line[0], line[1]), cv::Point(line[2], line[3])));
		else
			linRight.push_back(linePair(cv::Point(line[0], line[1]), cv::Point(line[2], line[3])));
	}

	drawLines(linLeft, cv::Scalar(0, 0, 255));
	drawLines(linRight, cv::Scalar(0, 255, 0));
	show();

}

inline void HoughLinesPR::doVerticalHough() {

	//if (!lines.empty())
	//	lines.clear();

	////cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);
	//HoughLines(image_, lines, 1, degree, threshold, 0, 0);

	////std::cout << "doVerticalHough # lines : " << lines.size();

	//if (lines.empty())
	//	return;

	//linePointsVert.clear();
	//linePointsVert.reserve(lines.size());

	//for (auto& l : lines) {
	//	auto theta = l[1];
	//	if (theta <= degree * (180 - angleLimit) && theta >= degree * angleLimit)
	//		continue;
	//	auto p = computeLinePair(l);
	//	linePointsVert.push_back(p);
	//	//std::cout << "vert : " << p.first << " " << p.second << endl;
	//}

	////std::cout << ' ' << linePointsVert.size() << " fits..\n";

	//drawLines(linePointsVert, cv::Scalar(255, 0, 255));
	//computeBorders();
}

inline void HoughLinesPR::doHorizontalHough() {

	//if (!lines.empty())
	//	lines.clear();

	//HoughLines(image_, lines, 1, degree, threshold, 0, 0);
	////cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);

	////std::cout << "doHorizontalHough # lines : " << lines.size();

	//if (lines.empty())
	//	return;

	//linePointsHori.clear();
	//linePointsHori.reserve(lines.size());

	//for (auto& l : lines) {
	//	auto theta = l[1];
	//	if (theta <= degree * 89.9 || theta >= degree * 90.1)
	//		continue;
	//	auto p = computeLinePair(l);
	//	if (p.first.y > original_.rows / 3)
	//		linePointsHori.push_back(p);
	//}

	////std::cout << ' ' << linePointsHori.size() << " fits..\n";

	//// quick test
	//auto sum = 0.0;
	//for (auto& l : linePointsHori) {
	//	sum += l.first.y;
	//}

	//sum /= linePointsHori.size();

	//leftY += sum;

	//drawLines(linePointsHori, cv::Scalar(255, 0, 0));
}

inline _cv::linePair HoughLinesPR::computeLinePair(cv::Vec2f& line) const {
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

inline void HoughLinesPR::drawLines(std::vector<linePair>& linePairs, cv::Scalar colour) {
	if (!showWindow)
		return;

	for (auto& r : linePairs) {
		line(output, r.first, r.second, colour, 1, CV_AA);
		//line(original, r.first, r.second, colour, 1, CV_AA);
	}
}

inline void HoughLinesPR::show() const {
	cv::imshow(windowName, output);
}
