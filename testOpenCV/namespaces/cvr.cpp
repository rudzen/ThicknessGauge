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

        return r;
    }

    double computeIntensityMean(cv::Mat& image) {

        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        auto m = cv::mean(channels[0]);

        return m[0];

    }

    cv::Vec2d intensityStdDev(cv::Mat& image) {

        cv::Scalar mean;
        cv::Scalar stdDev;
        cv::meanStdDev(image, mean, stdDev);

        return cv::Vec2d(mean[0], stdDev[0]);

    }

    cv::Vec4i getMinMaxLoc(cv::Mat& image, double minVal, double maxVal) {

        cv::Point minLoc;
        cv::Point maxLoc;

        cv::minMaxLoc(image, &minVal, &maxVal, &minLoc, &maxLoc);

        return cv::Vec4i(minLoc.x, minLoc.y, maxLoc.x, maxLoc.y);

    }

}
