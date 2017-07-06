//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include <string>
#include <opencv2/core/types.hpp>
#include "validate.h"
#include "sort.h"
#include "calc.h"

// helper functions for open cv related stuff.
namespace cvr {

    // helper functions to clear opencv structures without creating new objects
    namespace clear {

        template <typename T, int cn>
        void vec(cv::Vec<T, cn>& vec) {
            auto pos = cn;
            while (pos)
                vec[pos--] = static_cast<T>(0);
        }

        template <typename T>
        void rect(cv::Rect_<T>& rect) {
            rect.x = rect.y = rect.height = rect.width = static_cast<T>(0);
        }

        template <typename T>
        void point(cv::Point_<T>& p) {
            p.x = static_cast<T>(0);
            p.y = static_cast<T>(0);
        }

    }

    /**
    * \brief Adjust the marking rectangle based on the intersection points and specified buffer
    * \param marking_rect The rectangle to adjust
    * \param intersection_points The intersection points
    * \param buffer The buffer to apply
    */
    template <typename T>
    void adjust_marking_rect(cv::Rect_<T>& marking_rect, cv::Vec<T, 4>& intersection_points, T buffer) {
        static_assert(std::is_floating_point<T>::value, "Marking rectangles should only be treated as floating points.");
        marking_rect.x = intersection_points[0] + buffer;
        marking_rect.width = intersection_points[2] - marking_rect.x - buffer;
    }

    template <typename T1, typename T2>
    void avg_vector_vec(const std::vector<cv::Vec<T1, 4>>& vecvec, cv::Vec<T2, 4>& out) {
        static_assert(std::is_floating_point<T1>::value, "Wrong type.");
        static_assert(std::is_floating_point<T2>::value, "Wrong type.");
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
        out[3] = 0.0;
        for (auto& v : vecvec) {
            out[0] += v[0];
            out[1] += v[1];
            out[2] += v[2];
            out[3] += v[3];
        }
        out[0] /= vecvec.size();
        out[1] /= vecvec.size();
        out[2] /= vecvec.size();
        out[3] /= vecvec.size();
    }

    template <typename T1, typename T2>
    void avg_vecrect_x_width(const std::vector<cv::Rect_<T1>>& vecrec, cv::Rect_<T2>& out) {
        static_assert(std::is_floating_point<T1>::value, "Wrong type.");
        static_assert(std::is_floating_point<T2>::value, "Wrong type.");

        if (vecrec.empty()) {
            clear::rect(out);
            return;
        }

        out.x = 0.0;
        out.width = 0.0;
        auto count = 0;
        for (const auto& r : vecrec) {
            if (!validate::validate_rect(r))
                continue;
            out.x += r.x;
            out.width += r.width;
            count++;
        }
        out.x /= count;
        out.width /= count;

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
    void compute_intensity_std_dev(cv::Mat& image, cv::Vec<T, 2>& output) {
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

    /**
     * \brief Computes the gabs in a vector of points and populates them in target vector
     * \param elements The elements to fill gabs in
     * \param target The target vector of points
     * \param barrier The current known location of the baseLevel, default is 0, which means all gabs will be filled
     * \return  representing the number of elements filled in both X and Y
     */
    template <typename T>
    cv::Vec<T, 2> fill_pixel_gabs(cv::Point_<T>& elements, std::vector<cv::Point_<T>>& target, T barrier = 0) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible type");

        // set limits in elements to ensure gab finding in empty image
        elements.emplace_back(cv::Point_<T>(0, target.rows));
        elements.emplace_back(cv::Point_<T>(target.cols, target.rows));

        sorter::sort_pixels_x_ascending(elements);

        auto size = elements.size();

        elements.emplace_back(elements.front());

        // skip ahead to first element.
        auto first = elements.front().x;

        cv::Point_<T> sum;

        auto gab = false;

        for (auto i = first; i < size; ++i) {
            if (elements[i].y > target.rows - barrier)
                continue;
            if (i + 1 == size)
                break;
            auto x_pos = elements[i].x;
            auto x_next = elements[i + 1].x;
            auto dif = x_next - x_pos;
            if (dif < 2) {
                // no gab in x, try with y !!
                auto y_pos = elements[i].y;
                auto y_next = elements[i + 1].y;
                auto y_diff = y_next - y_pos;
                if (y_diff < 4)
                    continue;
                gab = true;
            }
            if (gab) {
                // construct line and populate output vector
                auto it = cv::LineIterator(cv::Mat(elements[i + 1].x - elements[i].x, calc::maxval(elements[i + 1].y, elements[i].y), CV_8U), elements[i], elements[i + 1], 8);
                for (auto& p : it)
                    target.emplace_back(p);

                sum.x += abs(elements[i + 1].x - elements[i].x);
                sum.y += abs(elements[i + 1].y - elements[i].y);
            }
            gab ^= true;
        }

        return cv::Vec<T, 2>(sum);
    }

    /**
     * \brief Gather all elements in vector of points that matches a specific X position
     * \tparam T1 Type of input points and X value
     * \tparam T2 Type of output points
     * \param input Input vector to look through
     * \param output Output vector to copy the elements to
     * \param x The X value to look for
     */
    template <typename T1, typename T2>
    void gather_elemenents_x(std::vector<cv::Point_<T1>>& input, std::vector<cv::Point_<T2>>& output, T1 x) {
        for (auto& e : input)
            if (e.x == x)
                output.emplace_back(e);
    }

    /**
     * \brief Gather all elements in an image that matches a specific X position
     * \tparam T1 Type of input points and X value
     * \tparam T2 Type of output points
     * \param image Input image to grab the pixels from
     * \param out Output vector to copy the elements to
     * \param x The X value to look for
     */
    template <typename T1, typename T2>
    void gather_elements_x(cv::Mat& image, std::vector<cv::Point_<T1>>& out, T2 x) {
        static_assert(std::is_arithmetic<T1>::value || std::is_arithmetic<T2>::value, "Invalid type for element extration.");
        static_assert(std::is_convertible<T1, T2>::value, "Incompatible types for element extraction.");
        cv::findNonZero(image(cv::Rect(x, 0, 1, image.rows)), out);
    }

    /**
     * \brief Gather all elements in vector of points that matches a specific Y position
     * \tparam T1 Type of input points and Y value
     * \tparam T2 Type of output points
     * \param input Input vector to look through
     * \param output Output vector to copy the elements to
     * \param y The Y value to look for
     */
    template <typename T1, typename T2>
    void gather_elemenents_y(std::vector<cv::Point_<T1>>& input, std::vector<cv::Point_<T2>>& output, T1 y) {
        for (auto& e : input)
            if (e.y == y)
                output.emplace_back(e);
    }

    /**
     * \brief Gather all elements in an image that matches a specific Y position
     * \tparam T1 Type of input points and Y value
     * \tparam T2 Type of output points
     * \param image Input image to grab the pixels from
     * \param out Output vector to copy the elements to
     * \param y The Y value to look for
     */
    template <typename T1, typename T2>
    void gather_elements_y(cv::Mat& image, std::vector<cv::Point_<T1>>& out, T2 y) {
        static_assert(std::is_arithmetic<T1>::value || std::is_arithmetic<T2>::value, "Invalid type for element extration.");
        static_assert(std::is_convertible<T1, T2>::value, "Incompatible types for element extraction.");
        cv::findNonZero(image(cv::Rect(0, y, image.cols, 1)), out);
    }

    /**
     * \brief Locates the highest pixel in the image
     * \param image The image to look in
     * \return The Y value of the highest located pixel as int
     */
    int highest_y_in_image(cv::Mat& image);

    /**
     * \brief Crude cutoff of pixels from image based on Y
     * \param image The image data
     * \param output The output vector
     * \param y_limit The limit in height
     * \return true if something was found, otherwise false
     */
    template <typename T>
    bool get_pix_above_y(cv::Mat& image, std::vector<cv::Point_<T>>& output, int y_limit) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible types for element extraction.");

        std::vector<cv::Point> result;
        cv::findNonZero(image, result);
        y_limit = abs(image.rows - y_limit);
        for (auto& p : result) {
            if (p.y <= y_limit)
                output.emplace_back(p);
        }
        return !output.empty();
    }

    /**
     * \brief Crude cutoff of pixels from image based on Y
     * \param pixels The image data in vector of pixels
     * \param target The output vector
     * \param y_limit The limit in height
     * \return true if something was found, otherwise false
     */
    template <typename T>
    bool get_pix_above_y(std::vector<cv::Point_<T>>& pixels, std::vector<cv::Point_<T>>& target, double y_limit, double image_height) {
        static_assert(std::is_arithmetic<T>::value, "Incompatible types for element extraction.");

        if (!target.empty())
            target.clear();

        y_limit = abs(image_height - y_limit);

        target.reserve(calc::round(y_limit));

        for (auto& p : pixels) {
            if (p.y <= y_limit)
                target.emplace_back(p);
        }

        return !target.empty();
    }

    /**
     * \brief Converts any type of rect to double type rect
     * \tparam T The type of rect
     * \param rect The rect to convert
     * \return The newly created double type rect
     */
    template <typename T>
    cv::Rect2d rect_any_to_double(cv::Rect_<T>& rect) {
        auto x = static_cast<double>(rect.x);
        auto y = static_cast<double>(rect.y);
        auto w = static_cast<double>(rect.width);
        auto h = static_cast<double>(rect.height);
        return cv::Rect2d(x, y, w, h);
    }

    /**
     * \brief Forcefully aligns a rectangle x and y values
     * Values can be negative because of float roundings.
     * \tparam T The type of rectangle
     * \tparam min_val The minimum value it should be
     * \param rect 
     */
    template <typename T, int min_val>
    void rect_force_align_boundries(cv::Rect_<T>& rect, T max_width, T max_height) {
        static_assert(std::is_arithmetic<T>::value, "Unsupported type.");
        if (rect.x < min_val)
            rect.x = min_val;
        if (rect.y < min_val)
            rect.y = min_val;

        if (rect.width > max_width)
            rect.width = max_width;
        if (rect.height > max_height)
            rect.height = max_height;
    }

    /**
     * \brief Split the original frames into two vectors based on the center of the matrix size in X.
     * Note that the resulting vectors only contains references to the original frames.
     * \param frames The frames to be split in half (verticaly)
     * \param left_out The output left side of the frames
     * \param right_out The output right side of the frames
     */
    void split_frames(std::vector<cv::Mat>& frames, std::vector<cv::Mat>& left_out, std::vector<cv::Mat>& right_out);

    /**
    * \brief Sums the intensity for a specific coloumn in a matrix
    * \param image The image matrix to sum from
    * \param x The X column to sum
    * \return The avg intensity for specified column
    */
    template <typename T1, typename T2>
    double sum_column_intensity(cv::Mat_<T1>& image, T2 x) {
        static_assert(std::is_arithmetic<T1>::value && std::is_integral<T2>::value, "invalid type");

        auto sum = 0;
        auto count = 0;

        for (auto col = 0; col < image.cols; ++col) {
            auto uc_pixel = image.data + x * image.step;
            T1 intensity = uc_pixel[0];
            if (intensity == 0)
                continue;
            sum += intensity;
            count++;
        }

        return sum / static_cast<double>(count);
    }

    /**
     * \brief Converts an image type to string
     * \param type The type to convert
     * \return The type as string
     */
    std::string type2str(int type);

}
