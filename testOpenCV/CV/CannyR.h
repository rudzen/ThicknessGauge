#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"
#include "../namespaces/pixel.h"
#include "../namespaces/tg.h"

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

/**
 * \brief Canny algorithm wrapper class
 */
class CannyR : public BaseR {

    cv::Mat edges_;

    int threshold_1_;

    int threshold_2_;

    const int APER_MIN = 3;

    const int APER_MAX = 7;

    int aperture_size_;

    int gradient_;

    bool remove_pepper_noise_;

    void createWindow() {
        namedWindow(window_name_, cv::WINDOW_KEEPRATIO);
        cv::createTrackbar("threshold1", window_name_, &threshold_1_, 200, threshold1cb, this);
        cv::createTrackbar("threshold2", window_name_, &threshold_2_, 200, threshold2cb, this);
        cv::createTrackbar("apertureSize", window_name_, &aperture_size_, APER_MAX, apertureSizecb, this);
        cv::createTrackbar("gradient", window_name_, &gradient_, 1, gradientcb, this);
    }

    static void threshold1cb(int value, void* userData);

    static void threshold2cb(int value, void* userData);

    static void apertureSizecb(int value, void* userData);

    static void gradientcb(int value, void* userData);

    void aperture_size(int new_aperture_size) {
        auto new_size = new_aperture_size;
        if (new_size < APER_MIN)
            new_size = APER_MIN;
        else if (new_size % 2 == 0)
            new_size++;

        aperture_size_ = new_size;
    }

public:

    CannyR(const int threshold_1, const int threshold_2, const int aperture_size, const bool gradient, const bool show_windows, const bool remove_pepper_noise)
        : BaseR("Canny", show_windows)
          , threshold_1_(threshold_1)
          , threshold_2_(threshold_2)
          , aperture_size_(aperture_size)
          , gradient_(gradient)
          , remove_pepper_noise_(remove_pepper_noise) {
        if (show_windows)
            createWindow();
    }

    void threshold_1(int threshold1) {
        threshold_1_ = threshold1;
    }

    void threshold_2(int threshold2) {
        threshold_2_ = threshold2;
    }

    void gradient(int gradient) {
        this->gradient_ = gradient;
    }

    void do_canny();

    cv::Mat& result();

};

/* UI callback functions */

inline void CannyR::threshold1cb(int value, void* user_data) {
    auto that = static_cast<CannyR*>(user_data);
    that->threshold_1(value);
    using namespace tg;
    log_time << cv::format("Canny threshold 1 : %i\n", value);
}

inline void CannyR::threshold2cb(int value, void* user_data) {
    auto that = static_cast<CannyR*>(user_data);
    that->threshold_2(value);
    using namespace tg;
    log_time << cv::format("Canny threshold 2 : %i\n", value);
}

inline void CannyR::apertureSizecb(int value, void* user_data) {
    auto that = static_cast<CannyR*>(user_data);
    that->aperture_size(value);
    using namespace tg;
    log_time << cv::format("Canny aperture size : %i\n", value);
}

inline void CannyR::gradientcb(int value, void* user_data) {
    auto that = static_cast<CannyR*>(user_data);
    that->gradient(value);
    std::string boltab[2] = {"false", "true"};
    using namespace tg;
    log_time << cv::format("Canny gradient : %s\n", boltab[static_cast<bool>(value)]);
}
