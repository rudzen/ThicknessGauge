//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "calc.h"

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

    /**
     * \brief Sorts a list of vectors in ascending order based on their angle
     * \tparam T Type of vec
     * \param vec_vecs The vector containing the vectors
     */
    template <typename T>
    void sort_vec_by_angle_ascending(std::vector<cv::Vec<T, 4>>& vec_vecs) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vec_vecs.begin(), vec_vecs.end(), [](const cv::Vec<T, 4>& v1, const cv::Vec<T, 4>& v2) {
            return calc::angle_between_lines(v1[0], v1[2], v1[1], v1[3]) < calc::angle_between_lines(v2[0], v2[2], v2[1], v2[3]);
        });
    }

    /**
     * \brief Sorts a list of vectors in decending order based on their angle
     * \tparam T Type of vec
     * \param vec_vecs The vector containing the vectors
     */
    template <typename T>
    void sort_vec_by_angle_descending(std::vector<cv::Vec<T, 4>>& vec_vecs) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type.");
        std::sort(vec_vecs.begin(), vec_vecs.end(), [](const cv::Vec<T, 4>& v1, const cv::Vec<T, 4>& v2) {
            return calc::angle_between_lines(v1[0], v1[2], v1[1], v1[3]) > calc::angle_between_lines(v2[0], v2[2], v2[1], v2[3]);
        });
    }

    template <typename T>
    bool search_x(std::vector<cv::Point_<T>>& vec, T x) {
        cv::Point_<T> p;
        return std::binary_search(vec.begin(), vec.end(), p, [&](cv::Point_<T>& p1, cv::Point_<T>& p2) {
            return p1.x == x;
        });
    }

    template <typename T>
    bool search_y(std::vector<cv::Point_<T>>& vec, T y) {
        cv::Point_<T> p;
        return std::binary_search(vec.begin(), vec.end(), [&](cv::Point_<T>& p1, cv::Point_<T>& p2) {
            return p1.y == y;
        });
    }


}
