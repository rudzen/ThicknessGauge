
//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/shape/hist_cost.hpp>

namespace sort {

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#elif defined(__GNUC__) && !defined(inline)
#define inline __always_inline
#endif

    template <typename T>
    inline
    void sort_contours(std::vector<std::vector<cv::Point_<T>>>& contours) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point_<T>> a, std::vector<cv::Point_<T>> b) { return contourArea(a) > contourArea(b); });
    }

    template <typename T>
    inline
    void sort_pixels_x_ascending(std::vector<cv::Point_<T>>& vector) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vector.begin(), vector.end(), [](cv::Point_<T> p1, cv::Point_<T> p2) { return p1.x < p2.x; });
    }

    template <typename T>
    inline
    void sort_pixels_x_descending(std::vector<cv::Point_<T>>& vector) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vector.begin(), vector.end(), [](cv::Point_<T> p1, cv::Point_<T> p2) { return p1.x > p2.x; });
    }

    template <typename T>
    inline
    void sort_pixels_y_ascending(std::vector<cv::Point_<T>>& vector) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vector.begin(), vector.end(), [](cv::Point_<T> p1, cv::Point_<T> p2) { return p1.y < p2.y; });
    }

    template <typename T>
    inline
    void sort_pixels_y_descending(std::vector<cv::Point_<T>>& vector) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vector.begin(), vector.end(), [](cv::Point_<T> p1, cv::Point_<T> p2) { return p1.y > p2.y; });
    }

}
