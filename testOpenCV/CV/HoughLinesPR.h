#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"
#include "Pixel.h"
#include "HoughLinesR.h"
#include <opencv2/core/affine.hpp>

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

	// the horizontal lines as vec4f
	vector<cv::Vec4f> linesHori;

	// the vertical lines as vec2f
	vector<cv::Vec2f> linesVert;

	vector<linePair> linePointsVert;

	vector<linePair> linePointsHori;

	vector<cv::Point2i> rightBorder;

	vector<cv::Point2i> leftBorder;

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
		namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("rho", windowName, &rho, 3, rhocb, this);
		cv::createTrackbar("theta", windowName, &theta, 180, thetacb, this);
		cv::createTrackbar("threshold", windowName, &threshold, 100, thresholdcb, this);
		cv::createTrackbar("min len", windowName, &minLineLen, 200, minLineLencb, this);
		cv::createTrackbar("max gab", windowName, &maxLineGab, 100, maxLineGabcb, this);
	}

	void computeBorders();

	linePair HoughLinesPR::computeLinePair(cv::Vec2f& line) const;

	double getAngle(cv::Vec4f& vec) const;

	double getAngle(cv::Point& p1, cv::Point& p2) const;

	double getAngle(int x1, int x2, int y1, int y2) const;

	static bool splitX(vector<cv::Vec4f>& source, vector<cv::Vec4f>& right, vector<cv::Vec4f>& left, double x, double yMin, double *leftCenter, double *rightCenter);

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

	void drawLines(vector<linePair>& linePairs, cv::Scalar colour);

	void drawLines(vector<cv::Vec4f>& lines, cv::Scalar colour);

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
	}

	const vector<cv::Vec4f>& getLines() const {
		return linesHori;
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


	line(output, bestLow, bestTop, cv::Scalar(0, 255, 255), 3);

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

inline void HoughLinesPR::doVerticalHough() {

	if (!linesHori.empty())
		linesHori.clear();

	//cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);
	HoughLines(image_, linesVert, 1, degree, threshold, 0, 0);

	//std::cout << "doVerticalHough # lines : " << lines.size();

	if (linesHori.empty())
		return;

	linePointsVert.clear();
	linePointsVert.reserve(linesHori.size());

	for (auto& l : linesVert) {
		auto theta = l[1];
		if (theta <= degree * (180 - 22.5) && theta >= degree * 22.5)
			continue;
		auto p = computeLinePair(l);
		linePointsVert.push_back(p);
		std::cout << "vert : " << p.first << " " << p.second << endl;
	}

	std::cout << ' ' << linePointsVert.size() << " fits..\n";

	drawLines(linePointsVert, cv::Scalar(255, 0, 255));
	computeBorders();
}

inline void HoughLinesPR::doHorizontalHough() {

	if (!linesHori.empty())
		linesHori.clear();

	linesHori.reserve(image_.cols * image_.rows);

	//std::cout << "b4 hlines" << std::endl;

	//HoughLinesP(image_, lines, 1, CV_PI / 180, 50, 50, 10);
	HoughLinesP(image_, linesHori, rho, CV_PI / 4, threshold, static_cast<double>(minLineLen), static_cast<double>(maxLineGab));
	//std::cout << "after hlines" << std::endl;

	auto count = linesHori.size();

	//std::cout << "HoughLinesP line count = " << count << std::endl;

	linePointsVert.clear();
	linePointsVert.reserve(count);

	linePointsHori.clear();
	linePointsHori.reserve(count);

	cv::Point2d center(image_.cols / 2, image_.rows / 2);
	cv::Point2d third(image_.cols / 3, image_.rows / 3);

	vector<linePair> linLeft;
	vector<linePair> linRight;

	linLeft.reserve(count);
	linRight.reserve(count);

	for (auto& line : linesHori) {

		// center horizontal cutoff
		if (line[1] < third.y || line[3] < third.y)
			continue;

		//calculate angle in radian,  if you need it in degrees just do angle * 180 / PI
		auto angle = getAngle(line);// atan2(line[0] - line[3], line[1] - line[2]);
		//cout << "Detected line " << line << " has angle " << angle * degree << endl;

		if (line[0] <= center.x)// && line[2] <= center.x)
			linLeft.push_back(linePair(cv::Point(line[0], line[1]), cv::Point(line[2], line[3])));
		else
			linRight.push_back(linePair(cv::Point(line[0], line[1]), cv::Point(line[2], line[3])));
	}

	//drawLines(linLeft, cv::Scalar(0, 0, 255));
	//drawLines(linRight, cv::Scalar(0, 255, 0));

	double centerRight[3] = { 0.0, 0.0, 0.0 };
	double centerLeft[3] = { 0.0, 0.0, 0.0 };

	vector<cv::Vec4f> left[3];
	vector<cv::Vec4f> right[3];

	auto split = splitX(linesHori, right[0], left[0], center.x, third.y * 2, &centerLeft[0], &centerRight[0]);

	//cout << "Center left  0 avg: " << centerLeft[0] << endl;
	//cout << "Center right 0 avg: " << centerRight[0] << endl;

	if (split) {
		if (centerRight[0] != 0.0)
			drawLines(right[0], cv::Scalar(255, 255, 0));

		if (centerLeft[0] != 0.0)
			drawLines(left[0], cv::Scalar(255, 0, 255));
	}

	// igen igen for HØJRE side

	split = splitX(left[0], right[1], left[1], centerLeft[0], third.y * 2, &centerLeft[1], &centerRight[1]);

	//cout << "Center left  1 avg: " << centerLeft[1] << endl;
	//cout << "Center right 1 avg: " << centerRight[1] << endl;

	if (split) {
		if (centerRight[1] != 0.0)
			drawLines(right[1], cv::Scalar(0, 0, 0));

		if (centerLeft[1] != 0.0)
			drawLines(left[1], cv::Scalar(255, 255, 255));
	}

	// for helvede.. igen for venstre

	split = splitX(right[0], right[2], left[2], centerRight[0], third.y * 2, &centerLeft[2], &centerRight[2]);

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

	auto maxX = right[1].back()[2];
	auto minX = right[1].front()[0];
	auto maxY = right[1].back()[1];
	auto minY = right[1].front()[1];

	for (auto& v : right[1]) {

		// check furthest point in X for max
		if (v[2] > maxX)
			maxX = v[2];

		// check closest point to 0 for min
		if (v[0] < minX)
			minX = v[0];

		sumX += v[0] + v[2];
		sumY += v[1] + v[3];
	}

	if (sumX != 0.0)
		sumX /= right[1].size();

	if (sumY != 0.0)
		sumY /= right[1].size();


	drawLine(0, cvRound(sumY), image_.cols, cvRound(sumY), cv::Scalar(0, 0, 0));
	//cout << "SumX: " << sumX << " SumY: " << sumY << " maxX: " << maxX << " maxY: " << maxY << endl;




	show();
}

inline linePair HoughLinesPR::computeLinePair(cv::Vec2f& line) const {
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

inline double HoughLinesPR::getAngle(cv::Vec4f& vec) const {
	return atan2(vec[1] - vec[3], vec[0] - vec[2]);
}

inline double HoughLinesPR::getAngle(cv::Point& p1, cv::Point& p2) const {
	return atan2(p1.y - p2.y, p1.x - p2.x);
}

inline double HoughLinesPR::getAngle(int x1, int x2, int y1, int y2) const {
	return atan2(y1 - y2, x1 - x2);
}

inline bool HoughLinesPR::splitX(vector<cv::Vec4f>& source, vector<cv::Vec4f>& right, vector<cv::Vec4f>& left, double x, double yMin, double *leftCenter, double *rightCenter) {

	*leftCenter = 0.0;
	*rightCenter = 0.0;

	for (auto& s : source) {
		if (s[1] >= yMin && s[3] >= yMin) { // desværre, ellers bliver størrelserne og dermed pointers fucked up.
			auto centerX = (s[2] + s[0]) / 2;
			if (centerX <= x) {
				left.push_back(s);
				*leftCenter += centerX;
			} else {
				right.push_back(s);
				*rightCenter += centerX;
			}
		}
	}

	if (!left.empty())
		*leftCenter /= left.size();

	if (!right.empty())
		*rightCenter /= right.size();

	return !(right.empty() && left.empty());

}

inline void HoughLinesPR::drawLines(vector<linePair>& linePairs, cv::Scalar colour) {
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
