#pragma once


#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

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
class HoughLinesR {

	typedef pair<cv::Point2i, cv::Point2i> linePair;

	cv::Mat original;

	cv::Mat image;

	cv::Mat output;

	std::vector<cv::Vec2f> lines;

	std::vector<linePair> linePointsVert;

	std::vector<linePair> linePointsHori;

	linePair rightBorder;

	linePair leftBorder;

	const std::string windowName = "HoughLines";

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

	double angleLimit;

	bool showWindow;

public:

	HoughLinesR(const int rho, const int theta, const int threshold, const bool showWindow)
		: rho(rho),
		  theta(theta),
		  threshold(threshold),
		  showWindow(showWindow) {
		angle = CV_PI / 180 * theta;
		srn = 0;
		stn = 0;
		minTheta = 0.0;
		maxTheta = CV_PI;
		angleLimit = 0;
		if (showWindow)
			createWindow();
	}

private:
	void createWindow() {
		cv::namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("rho", windowName, &rho, 3, rhocb, this);
		cv::createTrackbar("theta", windowName, &theta, 180, thetacb, this);
		cv::createTrackbar("threshold", windowName, &threshold, 100, thresholdcb, this);
		//cv::createTrackbar("srn", windowName, &srn, 1, srncb, this);
		//cv::createTrackbar("stn", windowName, &stn, 1, stncb, this);
	}

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
		angle = CV_PI / 180 * theta;
	}

	void setThreshold(int threshold) {
		this->threshold = threshold;
	}


public:

	void doVerticalHough();
	void doHorizontalHough();
	linePair HoughLinesR::computeLinePair(cv::Vec2f& line) const;
	void drawLines(std::vector<linePair>& linePairs, cv::Scalar colour);

	void setAngleLimit(double angleLimit) {
		this->angleLimit = angleLimit / 2;
	}

	const cv::Mat& getImage() const {
		return image;
	}

	void setImage(cv::Mat& newImage) {
		image = newImage;
	}

	void setOriginal(cv::Mat& newImage) {
		original = newImage;
		cv::cvtColor(original, output, CV_GRAY2BGR);
	}

	const std::vector<cv::Vec2f>& getLines() const {
		return lines;
	}

};

inline void HoughLinesR::rhocb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setTheta(value);
	std::cout << "Hough rho : " << value << std::endl;
}

inline void HoughLinesR::thetacb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setTheta(value);
	std::cout << "Hough theta : " << value << std::endl;
}

inline void HoughLinesR::thresholdcb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setThreshold(value);
	std::cout << "Hough threshold : " << value << std::endl;
}

inline void HoughLinesR::doVerticalHough() {

	if (!lines.empty())
		lines.clear();

	//cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);
	HoughLines(image, lines, 1, CV_PI / 180, 30, 0, 0);

	//std::cout << "doVerticalHough # lines : " << lines.size();

	if (lines.empty())
		return;

	linePointsVert.clear();
	linePointsVert.reserve(lines.size());

	for (auto& l : lines) {
		auto theta = l[1];
		if (theta <= CV_PI / 180 * (180 - angleLimit) && theta >= CV_PI / 180 * angleLimit)
			continue;
		auto p = computeLinePair(l);
		linePointsVert.push_back(p);
		//std::cout << "vert : " << p.first << " " << p.second << endl;
	}

	std::cout << ' ' << linePointsVert.size() << " fits..\n";

	drawLines(linePointsVert, cv::Scalar(255, 0, 255));
}

inline void HoughLinesR::doHorizontalHough() {

	if (!lines.empty())
		lines.clear();

	HoughLines(image, lines, 1, CV_PI / 180, 30, 0, 0);
	//cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);

	std::cout << "doHorizontalHough # lines : " << lines.size();

	if (lines.empty())
		return;

	linePointsHori.clear();
	linePointsHori.reserve(lines.size());

	for (auto& l : lines) {
		auto theta = l[1];
		if (theta <= CV_PI / 180 * 89.9 || theta >= CV_PI / 180 * 90.1)
			continue;
		auto p = computeLinePair(l);
		if (p.first.y > original.rows / 3)
			linePointsHori.push_back(p);		
	}

	std::cout << ' ' << linePointsHori.size() << " fits..\n";

	// quick test
	double sum = 0.0;
	for (auto& l : linePointsHori) {
		sum += l.first.y;
	}

	sum /= linePointsHori.size();

	for (auto& l : linePointsHori) {
		l.first.y = sum;
		l.second.y = sum;
	}


	drawLines(linePointsHori, cv::Scalar(255, 0, 0));
}

inline HoughLinesR::linePair HoughLinesR::computeLinePair(cv::Vec2f& line) const {
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

inline void HoughLinesR::drawLines(std::vector<linePair>& linePairs, cv::Scalar colour) {
	if (!showWindow)
		return;

	for (auto& r : linePairs) {
		line(output, r.first, r.second, colour, 1, CV_AA);
		line(original, r.first, r.second, colour, 1, CV_AA);
	}
	cv::imshow(windowName, output);
}
