//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <vector>
#include "tg.h"
#include "../CV/Data.h"

using namespace tg;

/**
 * \brief Contains validation function for various types and containers.
 * All functions here returns true if something passed validation.
 * There is plenty of room for adding more specific criteria of validation here.
 */
namespace validate {

    /**
     * \brief Validates a rectangle that its boundries are all valid numbers
     * \tparam T The type of rectangle
     * \param rect The rectangle to validate
     * \return true if its values are above 0, otherweise false
     */
    template <typename T>
    bool validate_rect(const cv::Rect_<T>& rect) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for fundamental types.");

        auto valid_rectangle = [rect]()->bool {
            return rect.width > 0.0 && rect.height > 0.0 && rect.x >= 0.0 && rect.y >= 0.0;
        };

        bool result = valid_rectangle();

        if (!result) {
            using namespace tg;
            log_err << __FUNCTION__ << " rectangle failed validation : " << rect << std::endl;
            return false;
        }

        return true;

    }

    /**
     * \brief Validates a rectangle based on the boundries of an image
     * \tparam T Type
     * \param rect The rectangle to validate
     * \param boundry The boundry matrix to compare the rectangle boundries to
     * \return true if the rectangle seems ok
     */
    template <typename T>
    bool validate_rect(const cv::Rect_<T>& rect, const cv::Mat& boundry) {
        static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

        auto valid_rectangle = [rect](cv::Rect_<T>& boundry)->bool {

            if (!validate_rect(rect) || !validate_rect(boundry)) {
                return false;
            }

            if (!validate_rect(rect)) {
                return false;
            }

            return (rect & boundry).area() == boundry.area();

        };

        return valid_rectangle(cv::Rect(cv::Point(0, 0), boundry.size()));

    }

    /**
     * \brief Validates a vector of points that all containing points are above zero in both x and y
     * \tparam T The type
     * \param vec The vector of points to validate
     * \return true if all pixels are above 0 in both x and y, otherwise false
     */
    template <typename T>
    bool valid_pix_vec(std::vector<cv::Point_<T>>& vec) {
        static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

        if (vec.empty()) {
            return false;
        }

        // could be done with checkRange too, but that would add another dependency from opencv
        auto it = find_if(vec.begin(), vec.end(), [](const cv::Point_<T>& p) {
                      return p.x >= 0 && p.y >= 0;
                  });

        return it != vec.end();

    }

    /**
     * \brief Validates a opencv vec type for negative values
     * \tparam T The type of the vector
     * \tparam cn The size of the vector
     * \param v The vector
     * \return true if all values are >= 0, otherwise false
     */
    template <typename T, int cn>
    bool valid_vec(const cv::Vec<T, cn>& v) {
        for (auto i = cn; i--;) {
            if (v[i] < 0) {
                return false;
            }
        }

        return true;
    }

    /**
     * \brief Validates the entirety of the data structure
     * \param data Pointer for the data
     * \return true if a-okay, otherwise false
     */
    template <typename T>
    bool valid_data(const std::shared_ptr<Data<T>>& data) {

        // logging is temporary !
        using namespace tg;
        using namespace std;

        log_time << "Data verification initiated.\n";

        size_t failures = 0;

        if (data->glob_name.empty()) {
            log_err << cv::format("Val: globName failed [%i], name = %s\n", ++failures, data->glob_name);
        }

        //if (data->cameraPtr == nullptr && data->glob_name == "camera") {
        //    log_err << cv::format("Val: cameraPtr failed [%i], == nullptr\n", ++failures);
        //}

        if (!valid_pix_vec(data->center_points)) {
            log_err << cv::format("Val: centerPoints failed [%i], size == %i\n", ++failures, data->center_points.size());
        }

        if (!valid_pix_vec(data->left_points)) {
            log_err << cv::format("Val: leftPoints failed [%i], size = %i\n", ++failures, data->left_points.size());
        }

        if (!valid_pix_vec(data->right_points)) {
            log_err << cv::format("rightPoints failed, size = %i\n", data->right_points.size());
        }

        if (!valid_vec(data->points_start)) {
            log_err << cv::format("pointsStart failed, data = %d,%d,%d\n", data->points_start[0], data->points_start[1], data->points_start[2]);
        }

        if (!valid_vec(data->left_border)) {
            log_err << cv::format("leftBorder failed, data = %d,%d,%d,%d\n", data->left_border[0], data->left_border[1], data->left_border[2], data->left_border[3]);
        }

        if (!valid_vec(data->right_border)) {
            log_err << cv::format("rightBorder failed, data = %d,%d,%d,%d\n", data->right_border[0], data->right_border[1], data->right_border[2], data->right_border[3]);
        }

        if (!valid_vec(data->center_line)) {
            log_err << cv::format("centerLine failed, data = %d,%d,%d,%d\n", data->center_line[0], data->center_line[1], data->center_line[2], data->center_line[3]);
        }

        if (data->left_avg < 0.0) {
            log_err << cv::format("leftAvg failed, data = %f\n", data->left_avg);
        }

        if (data->center_avg < 0.0) {
            log_err << cv::format("centerAvg failed, data = %f\n", data->center_avg);
        }

        if (data->right_avg < 0.0) {
            log_err << cv::format("rightAvg failed, data = %f\n", data->right_avg);
        }

        if (data->difference < 0.0) {
            log_err << cv::format("difference failed, data = %f\n", data->difference);
        }

        if (failures > 0) {
            log_err << cv::format("Val: Failures = %i\n", failures);
        } else {
            log_ok << "All tests passed.\n";
        }

        return failures == 0;
    }

}
