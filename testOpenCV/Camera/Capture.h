#pragma once
#include <memory>
#include <ostream>
#include <opencv2/core/base.hpp>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>

#include "CaptureInterface.h"
#include "../namespaces/tg.h"

using namespace tg;

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

    std::unique_ptr<CaptureSettings> settings = std::make_unique<CaptureSettings>();

    // The target stddev to set configuration for.
    double targetStdDev_;

    double delta_;

public:

    cv::VideoCapture cap;

    Capture() {
        targetStdDev_ = NAN;
        delta_ = NAN;
    }

    ~Capture() = default;

    void initialize() {
        cap.set(CV_CAP_PROP_SETTINGS, 1);
        initialize(5000.0, 0.0);
    }

    void initialize(double exposure, double gain) {
        cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);
        cap.set(CV_CAP_PROP_PVAPI_PIXELFORMAT, 0);
        exposure = alignMinValue(exposure);
        gain = alignMinValue(gain);
        setExposure(exposure);
        setGain(gain);
    }

    bool isOpen() const {
        return cap.isOpened();
    }

    void targetStdDev(const double target) {
        targetStdDev_ = target;
    }

    void deltaValue(const double delta) {
        delta_ = delta;
    }

    void setBrightness(double value) {
        value = alignMinValue(value);
        cap.set(CV_CAP_PROP_BRIGHTNESS, value);
        settings->brightness = value;
    }

    double getBrightness(bool repoll = true) const {
        if (repoll)
            settings->brightness = cap.get(CV_CAP_PROP_BRIGHTNESS);
        return settings->brightness;
    }

    void setContrast(double value) {
        value = alignMinValue(value);
        cap.set(CV_CAP_PROP_CONTRAST, value);
        settings->contrast = value;
    }

    double getContrast(bool repoll = true) const {
        if (repoll)
            settings->contrast = cap.get(CV_CAP_PROP_CONTRAST);
        return settings->contrast;
    }

    void setExposure(double value) {
        value = alignMinValue(value);
        cap.set(CV_CAP_PROP_EXPOSURE, value);
        settings->exposure = value;
    }

    double getExposure(bool repoll = true) const {
        if (repoll)
            settings->exposure = cap.get(CV_CAP_PROP_EXPOSURE);
        return settings->exposure;
    }

    void setGain(double value) {
        value = alignMinValue(value);
        cap.set(CV_CAP_PROP_GAIN, value);
        settings->gain = value;
    }

    double getGain(bool repoll = true) const {
        if (repoll)
            settings->gain = cap.get(CV_CAP_PROP_GAIN);
        return settings->gain;
    }

    void retrieveAllInfo() final;

    double detectExposure();

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
