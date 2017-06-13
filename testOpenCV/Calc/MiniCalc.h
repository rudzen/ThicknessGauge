#pragma once

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "LineConfig.h"
#include "../Util/Vec.h"

#include "../namespaces/tg.h"
#include "namespaces/sort.h"

using namespace std;
using namespace tg;

class MiniCalc {

public:
    MiniCalc();
    ~MiniCalc();

    static double varianceCoefficient(double*__restrict s, int mean) {
        return *s / mean * 100;
    }

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

    static double computeYSum(vi& elements) {
        auto sum = 0.0;
        for (auto& e : elements)
            sum += e.y;
        return sum;
    }

    double computeWeigthedY(vi& elements, int x) {

        vi temp;

        getElementsX(elements, temp, x);

        if (temp.empty())
            return 0.0;

        return computeYSum(temp) / static_cast<double>(temp.size());

    }

    static void getElementsX(vi& input, vi& output, int x) {
        for (auto& e : input) {
            if (e.x == x)
                output.emplace_back(e);
        }
    }

    static void getElementsY(vi& input, vi& output, int y) {
        for (auto& e : input) {
            if (e.y == y)
                output.emplace_back(e);
        }
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

    /**
     * \brief Computes the average (mean) intensity of the entire image
     * \param image The image to calculate meaned intensity of
     * \return the avg
     */
    double computeIntensityMean(cv::Mat& image) const {

        vector<cv::Mat> channels;
        cv::split(image, channels);
        auto m = cv::mean(channels[0]);

        return m[0];

    }

    /**
     * \brief Computes the average (mean) intensity and the standard deviation of the same of the entire image
     * \param image The image to calculate meaned intensity and standard deviation of the mean on
     * \return the avg as a 2d double precision float vector
     */
    static cv::Vec2d intensityStdDev(cv::Mat& image) {

        cv::Scalar mean;
        cv::Scalar stdDev;
        cv::meanStdDev(image, mean, stdDev);

        return cv::Vec2d(mean[0], stdDev[0]);

    }

    /**
     * \brief Retrieve the location of both the minimum and the maximum point in an image
     * \param image The image to perform the operation on
     * \param minVal The minimum value acceptable
     * \param maxVal The maximum value acceptable
     * \return 4d vector with both points
     */
    static cv::Vec4i getMinMaxLoc(cv::Mat& image, double minVal, double maxVal) {

        cv::Point minLoc;
        cv::Point maxLoc;

        cv::minMaxLoc(image, &minVal, &maxVal, &minLoc, &maxLoc);

        return cv::Vec4i(minLoc.x, minLoc.y, maxLoc.x, maxLoc.y);

    }

};
