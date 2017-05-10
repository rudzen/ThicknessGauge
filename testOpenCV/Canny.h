#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>


class Canny {
	
	struct images{
		cv::Mat image;
		cv::Mat edges;
	};

	images p;

	const std::string windowName = "Canny";

	int threshold1;

	int threshold2;

	int apertureSize;

	bool gradient;

	bool showWindow;

	void createWindow() {
		cv::namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("threshold1", windowName, &threshold1, 200, threshold1cb, this);
		cv::createTrackbar("threshold2", windowName, &threshold2, 200, threshold2cb, this);
		cv::createTrackbar("apertureSize", windowName, &apertureSize, 3, apertureSizecb, this);
		cv::createTrackbar("gradient", windowName, &apertureSize, 1, gradientcb, this);
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
		//this->apertureSize = 2 * apertureSize + 1;
		this->apertureSize = apertureSize;
	}

	void setGradient(bool gradient) {
		this->gradient = gradient;
	}

public:
	Canny(const int threshold1, const int threshold2, const int apertureSize, const bool gradient, const bool showWindow)
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
		return p.image;
	}

	void setImage(cv::Mat& newImage) {
		p.image = newImage;
	}

	const cv::Mat& getEdges() const {
		return p.edges;
	}

};

inline void Canny::threshold1cb(int value, void* userData) {
	auto that = static_cast<Canny*>(userData);
	that->setThreshold1(value);
	std::cout << "Canny threshold 1 : " << value << std::endl;
}

inline void Canny::threshold2cb(int value, void* userData) {
	auto that = static_cast<Canny*>(userData);
	that->setThreshold2(value);
	std::cout << "Canny threshold 2 : " << value << std::endl;
}

inline void Canny::apertureSizecb(int value, void* userData) {
	auto that = static_cast<Canny*>(userData);
	that->setApertureSize(value);
	std::cout << "Canny apertureSize : " << value << std::endl;
}

inline void Canny::gradientcb(int value, void* userData) {
	auto that = static_cast<Canny*>(userData);
	that->setGradient(value);
	std::string boltab[2] = { "false", "true" };
	std::cout << "Canny gradient : " <<  boltab[static_cast<bool>(value)] << std::endl;
}

inline void Canny::doCanny() {
	cv::Canny(p.image, p.edges, threshold1, threshold2, apertureSize, gradient);

	if (showWindow)
		cv::imshow(windowName, cv::WINDOW_KEEPRATIO);
}
