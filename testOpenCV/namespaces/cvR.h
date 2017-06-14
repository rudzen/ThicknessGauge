#pragma once
#include <string>
#include <opencv2/core/types.hpp>
#include "validate.h"

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#elif defined(__GNUC__) && !defined(inline)
#define inline __always_inline
#endif

// helper functions for open cv related stuff.
namespace cvr {

    std::string type2str(int type);

    /**
    * \brief Adjust the marking rectangle based on the intersection points and specified buffer
    * \param marking_rect The rectangle to adjust
    * \param intersection_points The intersection points
    * \param buffer The buffer to apply
    */
    template <typename T>
    inline
    void adjust_marking_rect(cv::Rect_<T>& marking_rect, cv::Vec<T, 4>& intersection_points, T buffer) {
        marking_rect.x = intersection_points[0] + buffer;
        marking_rect.width = intersection_points[2] - marking_rect.x - buffer;
    }

    template <typename T1, typename T2>
    inline
    void gather_elemenents_x(std::vector<cv::Point_<T1>>& input, std::vector<cv::Point_<T2>>& output, T1 x) {
        for (auto& e : input)
            if (e.x == x)
                output.emplace_back(e);
    }

    template <typename T1, typename T2>
    inline
    void gather_elemenents_y(std::vector<cv::Point_<T1>>& input, std::vector<cv::Point_<T2>>& output, T1 y) {
        for (auto& e : input)
            if (e.y == y)
                output.emplace_back(e);
    }

    /**
     * \brief Computes the average (mean) intensity of the entire image
     * \param image The image to calculate meaned intensity of
     * \return the avg
     */
    double computeIntensityMean(cv::Mat& image);

    /**
     * \brief Computes the average (mean) intensity and the standard deviation of the same of the entire image
     * \param image The image to calculate meaned intensity and standard deviation of the mean on
     * \return the avg as a 2d double precision float vector
     */
    cv::Vec2d intensityStdDev(cv::Mat& image);

    /**
     * \brief Retrieve the location of both the minimum and the maximum point in an image
     * \param image The image to perform the operation on
     * \param minVal The minimum value acceptable
     * \param maxVal The maximum value acceptable
     * \return 4d vector with both points
     */
    cv::Vec4i getMinMaxLoc(cv::Mat& image, double minVal, double maxVal);

}
