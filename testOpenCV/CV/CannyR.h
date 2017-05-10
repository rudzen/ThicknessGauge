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

class CannyR {
	
	cv::Mat image;

	cv::Mat edges;

	const std::string windowName = "Canny";

	int threshold1;

	int threshold2;

	const int aperMin = 3;
	const int aperMax = 7;
	int apertureSize;

	int gradient;

	bool showWindow;

	void createWindow() {
		cv::namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("threshold1", windowName, &threshold1, 200, threshold1cb, this);
		cv::createTrackbar("threshold2", windowName, &threshold2, 200, threshold2cb, this);
		cv::createTrackbar("apertureSize", windowName, &apertureSize, aperMax, apertureSizecb, this);
		cv::createTrackbar("gradient", windowName, &gradient, 1, gradientcb, this);
	}

	static void threshold1cb(int value, void *userData);
	static void threshold2cb(int value, void* userData);
	static void apertureSizecb(int value, void* userData);
	static void gradientcb(int value, void* userData);

	void setThreshold1(int threshold1) {
		this->threshold1 = threshold1;
	}

	void setThreshold2(int threshold2) {
		this->threshold2 = threshold2;
	}

	void setApertureSize(int apertureSize) {
		auto newSize = apertureSize;
		if (newSize < aperMin)
			newSize = aperMin;
		else if (newSize % 2 == 0)
			newSize++;

		this->apertureSize = newSize;
		//this->apertureSize = 2 * apertureSize + 1;
	}

	void setGradient(int gradient) {
		this->gradient = gradient;
	}

public:
	CannyR(const int threshold1, const int threshold2, const int apertureSize, const bool gradient, const bool showWindow)
		: threshold1(threshold1),
		  threshold2(threshold2),
		  apertureSize(apertureSize),
		  gradient(gradient),
		  showWindow(showWindow) {
		if (showWindow)
			createWindow();
	}

	void doCanny();

	const cv::Mat& getImage() const {
		return image;
	}

	void setImage(cv::Mat& newImage) {
		image = newImage;
	}

	const cv::Mat& getEdges() const {
		return edges;
	}

};

inline void CannyR::threshold1cb(int value, void* userData) {
	auto that = static_cast<CannyR*>(userData);
	that->setThreshold1(value);
	std::cout << "Canny threshold 1 : " << value << std::endl;
}

inline void CannyR::threshold2cb(int value, void* userData) {
	auto that = static_cast<CannyR*>(userData);
	that->setThreshold2(value);
	std::cout << "Canny threshold 2 : " << value << std::endl;
}

inline void CannyR::apertureSizecb(int value, void* userData) {
	auto that = static_cast<CannyR*>(userData);
	that->setApertureSize(value);
	std::cout << "Canny apertureSize : " << value << std::endl;
}

inline void CannyR::gradientcb(int value, void* userData) {
	auto that = static_cast<CannyR*>(userData);
	that->setGradient(value);
	std::string boltab[2] = { "false", "true" };
	std::cout << "Canny gradient : " <<  boltab[static_cast<bool>(value)] << std::endl;
}

inline void CannyR::doCanny() {

	cv::Canny(image, edges, threshold1, threshold2, apertureSize, gradient > 0);

	if (showWindow)
		cv::imshow(windowName, edges);
}
