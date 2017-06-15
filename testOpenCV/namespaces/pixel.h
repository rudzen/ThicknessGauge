#pragma once
#include <opencv2/core/mat.hpp>
#include "CV/LineConfig.h"
#include "calc.h"

namespace pixel {
    
#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#endif

    /**
     * \brief Get the intensity of a specific point on a specific image
     * \param image The image to get the intensity from
     * \param point The location of the point to get the intensity from
     * \return The intensity ranging from 0 to 255 (uchar)
     */
    template <typename T>
    inline
    uchar get_intensity(cv::Mat& image, cv::Point_<T>& point) {
        return image.at<uchar>(point);
    }

    /**
     * \brief Determine if a pixel is not zero and is contained inside the image boundries
     * \param image The image
     * \param pixel The point of the pixel
     * \return true if > 0 otherwise false
     */
    template <typename T>
    inline
    bool is_pixel(cv::Mat& image, cv::Point_<T>& pixel) {
        if (pixel.x > image.cols)
            return false;

        if (pixel.y > image.rows)
            return false;

        return get_intensity(image, pixel) > 0;
    }

    /**
     * \brief Get avg intensity for parsed line (0-255)
     * \param image The image to use as base for intensity computation
     * \param vector The vector to check for
     * \param connectivity Line connectivity, default = 8
     * \param left_to_right Reverse direction, default = false
     * \return The average intensity for the line, ranging 0-255 (uchar)
     */
    template <typename T>
    inline
    double getLineAvgIntensity(cv::Mat& image, cv::Vec<T, 4>& vector, int connectivity = 8, bool left_to_right = false) {

        cv::LineIterator it(image, cv::Point(calc::round(vector[0]), calc::round(vector[1])), cv::Point(calc::round(vector[2]), calc::round(vector[3])), connectivity, left_to_right);

        if (it.count == 0)
            return 0;

        auto sum = 0.0;
        for (auto i = 0; i < it.count; i++ , ++it) {
            auto pt = it.pos();
            sum += get_intensity(image, pt);
        }

        sum /= it.count;

        return sum;

    }

    /**
     * \brief Removed problematic 2 pixel radius black island pixels by filling with white
     * The intent for this function is to be used after a binary threshold
     * and to make certain algorithms work more smoothly as islands of pixels will be removed.
     * \param mask The image to use
     */
    void remove_pepper_noise(cv::Mat& mask);

}
