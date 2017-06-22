//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "cvR.h"
#include <opencv2/core/hal/interface.h>

namespace cvr {

    void split_frames(std::vector<cv::Mat>& frames, std::vector<cv::Mat>& left_out, std::vector<cv::Mat>& right_out) {

        cv::Point top_left(0, 0);
        cv::Point buttom_left(frames.front().cols / 2, frames.front().rows);

        cv::Point top_right(frames.front().cols / 2, 0);
        cv::Point buttom_right(frames.front().cols, frames.front().rows);

        cv::Rect left_rect(top_left, buttom_left);
        cv::Rect right_rect(top_right, buttom_right);

        for (auto& frame : frames) {
            left_out.emplace_back(frame(left_rect));
            right_out.emplace_back(frame(right_rect));
        }

    }

    std::string type2str(int type) {
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch (depth) {
            case CV_8U:
                r = "8U";
                break;
            case CV_8S:
                r = "8S";
                break;
            case CV_16U:
                r = "16U";
                break;
            case CV_16S:
                r = "16S";
                break;
            case CV_32S:
                r = "32S";
                break;
            case CV_32F:
                r = "32F";
                break;
            case CV_64F:
                r = "64F";
                break;
            default:
                r = "User";
                break;
        }

        r += "C";
        r += (chans + '0');

        return std::string(r);
    }

    int highest_y_in_image(cv::Mat& image) {

        std::vector<std::vector<cv::Point>> contours;

        findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        int highest_y = 0;;

        for (auto& c : contours) {
            auto r = cv::boundingRect(c);
            if (r.y > highest_y)
                highest_y = r.y;
        }

        return highest_y;
    }

}
