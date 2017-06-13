#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <memory>
#include "BaseR.h"
#include "Pixel.h"

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

class CannyR : public BaseR {
	
	cv::Mat edges_;

	std::shared_ptr<Pixelz> pPixelz;

	int threshold1_;

	int threshold2_;

	const int APER_MIN = 3;
	const int APER_MAX = 7;

	int apertureSize_;

	int gradient_;

	bool showWindow_;

	bool removePepperNoise_;

	void createWindow() {
		namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("threshold1", windowName, &threshold1_, 200, threshold1cb, this);
		cv::createTrackbar("threshold2", windowName, &threshold2_, 200, threshold2cb, this);
		cv::createTrackbar("apertureSize", windowName, &apertureSize_, APER_MAX, apertureSizecb, this);
		cv::createTrackbar("gradient", windowName, &gradient_, 1, gradientcb, this);
	}

	static void threshold1cb(int value, void *userData);
	static void threshold2cb(int value, void* userData);
	static void apertureSizecb(int value, void* userData);
	static void gradientcb(int value, void* userData);

	void setThreshold1(int threshold1) {
		this->threshold1_ = threshold1;
	}

	void setThreshold2(int threshold2) {
		this->threshold2_ = threshold2;
	}

	void setApertureSize(int apertureSize) {
		auto newSize = apertureSize;
		if (newSize < APER_MIN)
			newSize = APER_MIN;
		else if (newSize % 2 == 0)
			newSize++;

		this->apertureSize_ = newSize;
		//this->apertureSize = 2 * apertureSize + 1;
	}

	void setGradient(int gradient) {
		this->gradient_ = gradient;
	}

public:
	CannyR(const int threshold1, const int threshold2, const int apertureSize, const bool gradient, const bool showWindow, const bool removePepperNoise)
		: threshold1_(threshold1),
		  threshold2_(threshold2),
		  apertureSize_(apertureSize),
		  gradient_(gradient),
		  showWindow_(showWindow),
		  removePepperNoise_(removePepperNoise) {
		windowName = "Canny";
		if (showWindow)
			createWindow();
	}

	void doCanny();

	cv::Mat& getResult() {
		return edges_;
	}

	void setPixelz(const std::shared_ptr<Pixelz>& pixelz) {
		pPixelz = pixelz;
	}
};

inline void CannyR::threshold1cb(int value, void* user_data) {
	auto that = static_cast<CannyR*>(user_data);
	that->setThreshold1(value);
    log_time << cv::format("Canny threshold 1 : %i\n", value);
}

inline void CannyR::threshold2cb(int value, void* user_data) {
	auto that = static_cast<CannyR*>(user_data);
	that->setThreshold2(value);
    log_time << cv::format("Canny threshold 2 : %i\n", value);
}

inline void CannyR::apertureSizecb(int value, void* user_data) {
	auto that = static_cast<CannyR*>(user_data);
	that->setApertureSize(value);
    log_time << cv::format("Canny apertureSize : %i\n", value);
}

inline void CannyR::gradientcb(int value, void* user_data) {
	auto that = static_cast<CannyR*>(user_data);
	that->setGradient(value);
	string boltab[2] = { "false", "true" };
    log_time << cv::format("Canny gradient : %s\n", boltab[static_cast<bool>(value)]);
}

inline void CannyR::doCanny() {

	Canny(image_, edges_, threshold1_, threshold2_, apertureSize_, gradient_ > 0);

	if (removePepperNoise_)
		pPixelz->removePepperNoise(image_);
	
	if (showWindow_)
		imshow(windowName, edges_);
}
