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
        static_assert(std::is_floating_point<T>::value, "Marking rectangles should only be treated as floating points.");
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

    template <typename T1, typename T2>
    inline
    void gather_elements_y(cv::Mat& image, std::vector<cv::Point_<T1>>& out, T2 y) {
        static_assert(std::is_arithmetic<T1>::value || std::is_arithmetic<T2>::value, "Invalid type for element extration.");
        static_assert(std::is_convertible<T1, T2>::value, "Incompatible types for element extraction.");
        cv::findNonZero(image(cv::Rect(0, y, image.cols, 1)), out);
    }

    template <typename T1, typename T2>
    inline
    void gather_elements_x(cv::Mat& image, std::vector<cv::Point_<T1>>& out, T2 x) {
        static_assert(std::is_arithmetic<T1>::value || std::is_arithmetic<T2>::value, "Invalid type for element extration.");
        static_assert(std::is_convertible<T1, T2>::value, "Incompatible types for element extraction.");
        cv::findNonZero(image(cv::Rect(x, 0, 1, image.rows)), out);
    }

    /**
     * \brief Computes the average (mean) intensity of the entire image
     * \param image The image to calculate meaned intensity of
     * \return the avg
     */
    template <typename T>
    T compute_intensity_mean(cv::Mat& image) {
        static_assert(std::is_floating_point<T>::value, "Only floating point type makes sense.");

        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Scalar_<T> m = cv::mean(channels[0]);

        return m[0];

    }


    /**
     * \brief Computes the average (mean) intensity and the standard deviation of the same of the entire image
     * \param image The image to calculate meaned intensity and standard deviation of the mean on
     * \return the avg as a 2d double precision float vector
     */
    template <typename T>
    inline
    void compute_intensity_dtd_dev(cv::Mat& image, cv::Vec<T, 2>& output) {
        static_assert(std::is_floating_point<T>::value, "Only floating points makes sense");

        cv::Scalar mean;
        cv::Scalar std_dev;
        cv::meanStdDev(image, mean, std_dev);

        output[0] = mean[0];
        output[1] = std_dev[0];

    }

    /**
     * \brief Retrieve the location of both the minimum and the maximum point in an image
     *  http://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#minmaxloc
     * \param image The image to perform the operation on
     * \param minVal The minimum value acceptable
     * \param maxVal The maximum value acceptable
     * \return 4d vector with both points, 0 + 1 = first point, 2 + 3 = second point
     */
    template <typename T1, typename T2>
    inline
    cv::Vec<T1, 4> compute_min_max_loc(cv::Mat& image, T2 minVal, T2 maxVal) {
        static_assert(std::is_integral<T1>::value, "Only integral type is allowed.");
        static_assert(std::is_arithmetic<T2>::value, "Invalid type.");

        cv::Point minLoc;
        cv::Point maxLoc;

        auto min = static_cast<double>(minVal);
        auto max = static_cast<double>(maxVal);

        cv::minMaxLoc(image, &min, &max, &minLoc, &maxLoc);

        return cv::Vec<T1, 4>(minLoc.x, minLoc.y, maxLoc.x, maxLoc.y);

    }

}
