#pragma once

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "LineConfig.h"
#include "../Util/Vec.h"

#include "../namespaces/tg.h"
#include "namespaces/sort.h"
#include "namespaces/cvR.h"

using namespace std;
using namespace tg;

class MiniCalc {

public:
    MiniCalc();
    ~MiniCalc();

    /* Converts pixels from an image to a singular line vector in X, where Y is the mean of all pixels located at that given X in the image */
    bool generatePlanarPixels(cv::Mat& input, cv::Mat& output, vector<cv::Point2f>& pixels, vector<cv::Point2f>& gradientPixels) const;

    /**
     * \brief Computes a line from a vector of pixels points
     * \param pixels The pixels to calculate the line from
     * \param result The resulting line
     * \return true if line was created, otherwise false
     */
    static bool computerCompleteLine(vi& pixels, cv::Vec4f& result, LineConfig& config) {
        cv::fitLine(pixels, result, config.getDistType(), config.getParams(), config.getReps(), config.getAepa());
        return true;
    }

    /**
     * \brief Crude cutoff of pixels from image based on Y
     * \param image The image data
     * \param output The output vector
     * \param y_limit The limit in height
     * \return true if something was found, otherwise false
     */
    static bool getActualPixels(cv::Mat& image, vi& output, int y_limit);

    static bool getActualPixels(vi& pixels, vi& target, double y_limit, double image_height);






};
