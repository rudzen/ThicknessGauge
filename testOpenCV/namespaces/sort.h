//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

namespace sorter {

    /**
     * \brief Sorts vector of vector of points (contour structure), in descending order
     * \tparam T The type of point
     * \param contours The contours
     */
    template <typename T>
    void sort_contours(std::vector<std::vector<cv::Point_<T>>>& contours) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point_<T>>& a, const std::vector<cv::Point_<T>>& b) {
              return contourArea(a) > contourArea(b);
          });
    }

    /**
     * \brief Sorts a vector of points with respect to X in ascending order
     * \tparam T The type of point
     * \param vector The vector of points to sort
     */
    template <typename T>
    void sort_pixels_x_ascending(std::vector<cv::Point_<T>>& vector) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vector.begin(), vector.end(), [](const cv::Point_<T>& p1, const cv::Point_<T>& p2) {
              return p1.x < p2.x;
          });
    }

    /**
     * \brief Sorts a vector of points with respect to X in descending order
     * \tparam T The type of point
     * \param vector The vector of points to sort
     */
    template <typename T>
    void sort_pixels_x_descending(std::vector<cv::Point_<T>>& vector) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vector.begin(), vector.end(), [](const cv::Point_<T>& p1, const cv::Point_<T>& p2) {
              return p1.x > p2.x;
          });
    }

    /**
     * \brief Sorts a vector of points with respect to Y in ascending order
     * \tparam T The type of point
     * \param vector The vector of points to sort
     */
    template <typename T>
    void sort_pixels_y_ascending(std::vector<cv::Point_<T>>& vector) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vector.begin(), vector.end(), [](const cv::Point_<T>& p1, const cv::Point_<T>& p2) {
              return p1.y < p2.y;
          });
    }

    /**
     * \brief Sorts a vector of point with respect to Y in descending order
     * \tparam T The type of point
     * \param vector The vector of points to sort
     */
    template <typename T>
    void sort_pixels_y_descending(std::vector<cv::Point_<T>>& vector) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vector.begin(), vector.end(), [](const cv::Point_<T> p1, const cv::Point_<T>& p2) {
              return p1.y > & p2.y;
          });
    }

}
