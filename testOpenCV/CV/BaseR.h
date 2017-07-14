#pragma once
#include <string>

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
 * \brief Basic class for algorithm wrapping.
 */
class BaseR {

protected:

    /**
    * \brief The original un-modified image
    */
    cv::Mat original_;

    /**
    * \brief The image to process
    */
    cv::Mat image_;

    cv::Rect2d marking_rect_;

    int image_offset_ = 0;

    std::string window_name_;

    bool show_windows_ = false;

    BaseR(std::string windowName, bool showWindows)
        : window_name_(windowName)
          , show_windows_(showWindows) {}

public:

    BaseR() { }

    void original(const cv::Mat& original) {
        original_ = original;
    }

    cv::Mat& image() {
        return image_;
    }

    void image(const cv::Mat& image) {
        image_ = image;
    }

    cv::Rect2d marking_rect() const {
        return marking_rect_;
    }

    void marking_rect(const cv::Rect2d& markingRect) {
        this->marking_rect_ = markingRect;
    }

    bool show_windows() const {
        return show_windows_;
    }

    void show_windows(bool showWindows) {
        show_windows_ = showWindows;
    }
};
