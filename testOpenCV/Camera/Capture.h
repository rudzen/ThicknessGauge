// Rudy Alex Kohn

#pragma once
#include <memory>
#include <opencv2/videoio.hpp>
#include <ostream>
#include <opencv2/videoio/videoio_c.h>
#include <iostream>
#include "CaptureInterface.h"
#include <opencv2/imgcodecs.hpp>

/**
 * \brief Wrapper for generic camera capture.
	// PVAPI
    CV_CAP_PROP_PVAPI_MULTICASTIP           = 300, // ip for anable multicast master mode. 0 for disable multicast
    CV_CAP_PROP_PVAPI_FRAMESTARTTRIGGERMODE = 301, // FrameStartTriggerMode: Determines how a frame is initiated
    CV_CAP_PROP_PVAPI_DECIMATIONHORIZONTAL  = 302, // Horizontal sub-sampling of the image
    CV_CAP_PROP_PVAPI_DECIMATIONVERTICAL    = 303, // Vertical sub-sampling of the image
    CV_CAP_PROP_PVAPI_BINNINGX              = 304, // Horizontal binning factor
    CV_CAP_PROP_PVAPI_BINNINGY              = 305, // Vertical binning factor
    CV_CAP_PROP_PVAPI_PIXELFORMAT           = 306, // Pixel format
 */
class Capture : public CaptureInterface {

private:
	cv::VideoCapture& cap_;

	std::unique_ptr<CaptureSettings> settings = std::make_unique<CaptureSettings>();

	// The target stddev to set configuration for.
	double targetStdDev_;

	double delta_;

	static double alignValue(double value) {
		if (value < 0.0)
			return 0.0;
		return value;
	}

public:

	explicit Capture(cv::VideoCapture& cap) : cap_(cap) {
		targetStdDev_ = NAN;
		delta_ = NAN;

		// set default values
		setExposure(5000);
		setGain(0.0);
	}

	~Capture() = default;

	void targetStdDev(const double target) {
		targetStdDev_ = target;
	}

	void deltaValue(const double delta) {
		delta_ = delta;
	}

	void setBrightness(double value) const {
		value = alignValue(value);
		cap_.set(CV_CAP_PROP_BRIGHTNESS, value);
		settings->brightness = value;
	}

	double getBrightness(bool repoll = true) const {
		if (repoll) settings->brightness = cap_.get(CV_CAP_PROP_BRIGHTNESS);
		return settings->brightness;
	}

	void setContrast(double value) const {
		value = alignValue(value);
		cap_.set(CV_CAP_PROP_CONTRAST, value);
		settings->contrast = value;
	}

	double getContrast(bool repoll = true) const {
		if (repoll) settings->contrast = cap_.get(CV_CAP_PROP_CONTRAST);
		return settings->contrast;
	}

	void setExposure(double value) const {
		value = alignValue(value);
		cap_.set(CV_CAP_PROP_EXPOSURE, value);
		settings->exposure = value;
	}

	double getExposure(bool repoll = true) const {
		if (repoll) settings->exposure = cap_.get(CV_CAP_PROP_EXPOSURE);
		return settings->exposure;
	}

	void setGain(double value) const {
		value = alignValue(value);
		cap_.set(CV_CAP_PROP_GAIN, value);
		settings->gain = value;
	}

	double getGain(bool repoll = true) const {
		if (repoll) settings->gain = cap_.get(CV_CAP_PROP_GAIN);
		return settings->gain;
	}

	void retrieveAllInfo() final {
		settings->brightness = cap_.get(CV_CAP_PROP_BRIGHTNESS);
		settings->contrast = cap_.get(CV_CAP_PROP_CONTRAST);
		settings->saturation = cap_.get(CV_CAP_PROP_SATURATION);
		settings->hue = cap_.get(CV_CAP_PROP_HUE);
		settings->gain = cap_.get(CV_CAP_PROP_GAIN);
		settings->exposure = cap_.get(CV_CAP_PROP_EXPOSURE);
		settings->Rgb = cap_.get(CV_CAP_PROP_CONVERT_RGB);
		settings->white_balance_u = cap_.get(CV_CAP_PROP_WHITE_BALANCE_BLUE_U);
		settings->white_balance_v = cap_.get(CV_CAP_PROP_WHITE_BALANCE_RED_V);
		settings->rectification = cap_.get(CV_CAP_PROP_RECTIFICATION);
		settings->iso_speed = cap_.get(CV_CAP_PROP_ISO_SPEED);
		settings->buffersize = cap_.get(CV_CAP_PROP_BUFFERSIZE);
	}

	auto detectExposure() {
		cv::Mat t;
		cv::Scalar mean;
		cv::Scalar stddev(0.0, 0.0, 0.0);

		auto exposure = 0.0;

		cap_.set(CV_CAP_PROP_EXPOSURE, exposure);

		const cv::Vec2d target(targetStdDev_ - delta_, targetStdDev_ + delta_);

		while (stddev[0] < target[0]) {
			exposure += 50.0;
			cap_.set(CV_CAP_PROP_EXPOSURE, exposure);
			cap_ >> t;
			cv::meanStdDev(t, mean, stddev);

			std::cout << cv::format("trying [exposure:%f] [mean:%f] [stddev:%f]\n", exposure, mean[0], stddev[0]);

			imwrite(cv::format("images/exposuretest_%f.png", exposure), t);


		}

		std::cout << cv::format("BINGO [exposure:%f] [mean:%f] [stddev:%f]\n", exposure, mean[0], stddev[0]);

		return exposure;
	}





	friend std::ostream& operator<<(std::ostream& os, const Capture& obj) {
		os << "Capture device settings :\n";
		os << cv::format("brightness: %f\n", obj.settings->brightness);
		os << cv::format("contrast: %f\n", obj.settings->contrast);
		os << cv::format("saturation: %f\n", obj.settings->saturation);
		os << cv::format("hue: %f\n", obj.settings->hue);
		os << cv::format("gain: %f\n", obj.settings->gain);
		os << cv::format("exposure: %f\n", obj.settings->exposure);
		os << cv::format("Rgb: %f\n", obj.settings->Rgb);
		os << cv::format("white_balance_u: %f\n", obj.settings->white_balance_u);
		os << cv::format("white_balance_v: %f\n", obj.settings->white_balance_v);
		os << cv::format("rectification: %f\n", obj.settings->rectification);
		os << cv::format("iso_speed: %f\n", obj.settings->iso_speed);
		os << cv::format("buffersize: %f\n", obj.settings->buffersize);
		os << cv::format("Brightness: %f\n", obj.settings->brightness);
		os << cv::format("Brightness: %f\n", obj.settings->brightness);
		os << cv::format("Brightness: %f\n", obj.settings->brightness);
		return os;
	}
};
