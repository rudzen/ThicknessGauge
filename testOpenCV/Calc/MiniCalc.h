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

    int highestPixelInLine(cv::Mat& image) const;

    /* Converts pixels from an image to a singular line vector in X, where Y is the mean of all pixels located at that given X in the image */
    bool generatePlanarPixels(cv::Mat& input, cv::Mat& output, vector<cv::Point2f>& pixels, vector<cv::Point2f>& gradientPixels) const;

    /**
     * \brief Fills the gabs in the elements with simple linear lines
     * \param elements The elements to fill gabs in
     * \param target The target matrix
     * \param barrier The current known location of the baseLevel, default is 0, which means all gabs will be filled
     * \return Vector (2D) representing the number of elements filled in both X and Y
     */
    v2<double> fillElementGabs(vi& elements, cv::Mat& target, int barrier = 0) const {

        elements.emplace_back(cv::Point(0, target.rows));
        elements.emplace_back(cv::Point(target.cols, target.rows));

        sort::sort_pixels_x_ascending(elements);

        auto size = elements.size();

        elements.emplace_back(elements.front());

        auto first = elements.front().x;

        vector<cv::Point> gabLine;

        v2<double> sum;
        auto gab = false;

        for (auto i = first; i < size; ++i) {
            if (elements[i].y > target.rows - barrier)
                continue;
            if (i + 1 == size)
                break;
            auto posX = elements[i].x;
            auto nextX = elements[i + 1].x;
            auto dif = nextX - posX;
            if (dif < 2) {
                // no gab in x, try with y !!
                auto posY = elements[i].y;
                auto nextY = elements[i + 1].y;
                auto diffY = nextY - posY;
                if (diffY < 4)
                    continue;
                gab = true;
            }
            if (gab) {
                line(target, elements[i], elements[i + 1], cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                sum.x += abs(elements[i + 1].x - elements[i].x);
                sum.y += abs(elements[i + 1].y - elements[i].y);
            }
            gab ^= true;
        }

        return sum;
    }

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
