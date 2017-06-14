#include "cvR.h"
#include <opencv2/core/hal/interface.h>

namespace cvr {

    std::string type2str(int type) {
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch (depth) {
        case CV_8U: r = "8U";
            break;
        case CV_8S: r = "8S";
            break;
        case CV_16U: r = "16U";
            break;
        case CV_16S: r = "16S";
            break;
        case CV_32S: r = "32S";
            break;
        case CV_32F: r = "32F";
            break;
        case CV_64F: r = "64F";
            break;
        default: r = "User";
            break;
        }

        r += "C";
        r += (chans + '0');

        return std::string(r);
    }

    double computeIntensityMean(cv::Mat& image) {

        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        auto m = cv::mean(channels[0]);

        return m[0];

    }

    template <typename T>
    inline
    cv::Vec<T, 2> intensityStdDev(cv::Mat& image) {
        static_assert(std::is_floating_point<T>::value, "Only floating points makes sense");

        cv::Scalar mean;
        cv::Scalar stdDev;
        cv::meanStdDev(image, mean, stdDev);

        return cv::Vec<T, 2>(mean[0], stdDev[0]);

    }

    template <typename T>
    inline
    cv::Vec<T, 4> getMinMaxLoc(cv::Mat& image, double minVal, double maxVal) {
        static_assert(std::is_integral<T>::value, "Only integral type is allowed.");

        cv::Point minLoc;
        cv::Point maxLoc;

        cv::minMaxLoc(image, &minVal, &maxVal, &minLoc, &maxLoc);

        return cv::Vec<T, 4>(minLoc.x, minLoc.y, maxLoc.x, maxLoc.y);

    }

}
