//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/videostab/inpainting.hpp>

#include "tg.h"
#include "stl.h"
#include "validate.h"
#include "../CV/LineConfig.h"
#include "sort.h"

#ifndef CV_VERSION
#include "Util/Vec.h"
#endif

/**
 * \brief Calculation utility functionality
 * Contains optional OpenCV overloads for quick access
 */
namespace calc {

    constexpr double PI = 3.1415926535897932384626433832795;

    constexpr double PI_2 = PI / 2.0;

    constexpr double PI_4 = PI / 4.0;

    constexpr double DEGREES = PI / 180.0;

    constexpr double RADIANS = 180.0 / PI;

    constexpr double PIx2 = 2.0 * PI;

    constexpr double LOG2 = 0.69314718055994530941723212145818;

    /**
     * Round to the nearest integer
     * @param value The value to round
     * @return Nearest integer as double
     */
    template <typename T>
    int round(T value) {
        static_assert(std::is_floating_point<T>::value, "round is only possible for floating points.");
#if ((defined _MSC_VER && defined _M_X64) || (defined __GNUC__ && defined __x86_64__ \
    && defined __SSE2__ && !defined __APPLE__) || CV_SSE2) && !defined(__CUDACC__)
		__m128d t = _mm_set_sd(value);
		return _mm_cvtsd_si32(t);
#elif defined _MSC_VER && defined _M_IX86
        int t;
        __asm
        {
            fld value;
            fistp t;
        }
        return t;
#elif ((defined _MSC_VER && defined _M_ARM) || defined CV_ICC || \
        defined __GNUC__) && defined HAVE_TEGRA_OPTIMIZATION
		TEGRA_ROUND_DBL(value);
#elif defined CV_ICC || defined __GNUC__
# if defined ARM_ROUND_DBL
		ARM_ROUND_DBL(value);
# else
		return static_cast<int>(lrint(value));
# endif
#else
        /* it's ok if round does not comply with IEEE754 standard;
        the tests should allow +/-1 difference when the tested functions use round */
#ifdef CV_VERSION
		return static_cast<int>(cv::floor(d + 0.5));
#else
		return static_cast<int>(floor(d + 0.5));
#endif

#endif
    }

    /**
     * \brief Contains line specific computational functions
     */
    namespace line {

        /**
         * \brief Adjusts the baselines according to intersection points and specified buffer
         * \tparam T1 Type one, should be, must be floating point
         * \tparam T2 Type two, must be identicala type to Type one
         * \param base_lines The baseline
         * \param intersection_points The intersection points
         * \param buffer The buffer to add to left side and substract from right side
         */
        template <typename T1, typename T2>
        void adjust_base_lines_x(cv::Vec<T1, 4>& base_lines, cv::Vec<T1, 4>& intersection_points, T2 buffer) {
            static_assert(std::is_floating_point<T1>::value, "Requires floating point.");
            static_assert(std::is_same<T2, double>::value, "Required double floating point.");

            base_lines[0] = intersection_points[0] - buffer;
            base_lines[2] = intersection_points[2] + buffer;
        }

        /**
         * \brief Computes the minimum houghline lenght for properlistic houghline
         * \tparam T Type, only floating points
         * \param min_len The minimim length of the line
         * \param rect The rectangle of the marking location
         * \return  the computed value, but not less than min_len
         */
        template <typename T>
        double compute_houghp_min_line(T min_len, cv::Rect_<T>& rect) {
            static_assert(std::is_floating_point<T>::value, "Only floating point type allowed.");
            auto min_line_len = rect.width / static_cast<double>(32);
            return align_min_value(min_line_len, min_len);
        }

        /**
         * \brief Computes a line from a vector of pixels points
         * \param pixels The pixels to calculate the line from
         * \param result The resulting line as 4 point float vec
         * \return true if line was created, otherwise false
         */
        template <typename T>
        bool compute_line_fitting(std::vector<cv::Point_<T>>& pixels, cv::Vec4f& result, LineConfig& config) {
            cv::Vec4f results;
            cv::fitLine(pixels, results, config.dist_type(), config.params(), config.reps(), config.aeps());
            if (!validate::valid_vec<float, 4>(results))
                return false;
            result = results;
            return true;
        }

    }

    /**
     * \brief Contains pixel specific computational functions
     */
    namespace pixels {

        /**
         * \brief Converts pixels from an image to a singular line vector in X, where Y is the mean of all pixels located at that given X in the image
         * \tparam T1 Type of pixels
         * \tparam T2 Type of gradient pixels
         * \param input The input matrix
         * \param output The output matrix
         * \param pixels The pixels
         * \param gradient_pixels The gradient pixels
         * \return true if okay, otherwise false
         */
        template <typename T1, typename T2>
        [[deprecated("not really used anymore, but could still prove useful in the future")]]
        bool generate_planar_pixels(cv::Mat& input, cv::Mat& output, std::vector<cv::Point_<T1>>& pixels, std::vector<cv::Point_<T2>>& gradient_pixels) {

            static_assert(std::is_arithmetic<T1>::value, "Wrong type.");
            static_assert(std::is_floating_point<T2>::value, "Wrong type.");


            std::vector<cv::Point> pix;

            pix.reserve(input.cols);

            findNonZero(input, pix);

            gradient_pixels.clear();
            gradient_pixels.reserve(input.cols);
            for (auto x = input.cols; x--;)
                gradient_pixels.emplace_back(cv::Point_<T2>(0, 0));

            pixels.clear();
            pixels.reserve(input.cols);

            // sort the list in X
            sorter::sort_pixels_x_ascending(pix);

            auto x = pix.front().x;
            auto count = 0;
            auto y_sum = 0;
            auto y_mean = 0.0;
            auto gradient_sum = 0.0;

            for (const auto& p : pix) {
                if (p.x != x) {
                    if (count > 0) {
                        pixels.emplace_back(cv::Point(x, static_cast<int>(round(y_sum / static_cast<double>(count)))));
                        auto gradient = static_cast<unsigned char>(round(gradient_sum / count));
                        output.at<unsigned char>(pixels.back()) = gradient;
                        gradient_pixels[x].y = gradient;
                        count = 0;
                    }
                    y_sum = 0;
                }
                x = p.x;
                y_sum += p.y;
                y_mean += p.y;
                gradient_sum += input.at<unsigned char>(p);
                count++;
            }

            if (pixels.empty())
                return false;

            y_mean /= pixels.size();

            return true;
        }

    } // namespace pixels

    /**
     * \brief Contains lens specific computational functions
     */
    namespace lens {

        /**
         * Calculates the angular field of view based on the dimension and the focal length
         * @param dimension The dimension in mm
         * @param focal_lenght The focal length
         * @return The angular fov in degrees
         */
        template <typename T>
        double angular_fov(T dimension, T focal_lenght) {
            static_assert(std::is_floating_point<T>::value, "Wrong type.");
            return 2 * atan(dimension) / (2 * focal_lenght);
        }

        /**
         * Calculates the effective focal length based on magninification
         * @param focal_length The base focal length
         * @param magnification The maginifaction factor
         * @return The effective focal length
         */
        template <typename T>
        double effective_focal_lenght(T focal_length, T magnification) {
            static_assert(std::is_floating_point<T>::value, "Wrong type.");
            return focal_length * (1.0 + magnification);
        }

        /**
         * Calculate the angular extend of the target based on the target dimension and the focal length
         * @param target_dimension The target dimension
         * @param focal_length The focal length
         * @return The angular extend of the target
         */
        template <typename T>
        double angular_extend(T target_dimension, T focal_length) {
            static_assert(std::is_floating_point<T>::value, "Wrong type.");
            return 2 * atan(target_dimension / 2 * focal_length);
        }

        /**
         * Calculates camera field of view, based on the angular extend, image dimension and the image of the target dimension
         * @param angular_extend The angular extend
         * @param image_dimension The image dimension
         * @param image_of_target_dimension The image of target dimension
         * @return The camera field of view
         */
        template <typename T>
        double camera_fov(T angular_extend, T image_dimension, T image_of_target_dimension) {
            static_assert(std::is_floating_point<T>::value, "Wrong type.");
            return angular_extend * image_dimension / image_of_target_dimension;
        }

        /**
         * Calculates camera field of view for rectilinear lensing, based on the angular extend, image dimension and the image of the target dimension
         * @param target_dimension
         * @param focal_length
         * @param image_dimension
         * @param image_of_target_dimension
         * @return
         */
        template <typename T>
        double camera_fov(T target_dimension, T focal_length, T image_dimension, T image_of_target_dimension) {
            static_assert(std::is_floating_point<T>::value, "Wrong type.");
            return 2.0 * atan(target_dimension * image_dimension / 2.0 * focal_length * image_of_target_dimension);
        }

        /**
         * Calculate the rectilinear field of view with magnification
         * @param frame_size The frame size
         * @param focal_length The focal length
         * @param magnification The magnification
         * @return The calculated field of view
         */
        template <typename T>
        double fov_rectilinear(T frame_size, T focal_length, T magnification) {
            static_assert(std::is_floating_point<T>::value, "Wrong type.");
            return 2.0 * atan(frame_size / (focal_length * 2.0 * (magnification + 1.0)));
        }

        /**
         * Calculate the rectilinear field of view
         * @param frame_size The frame size
         * @param focal_length The focal length
         * @return The calculated field of view
         */
        template <typename T>
        double fov_rectilinear(T frame_size, T focal_length) {
            static_assert(std::is_floating_point<T>::value, "Wrong type.");
            return fov_rectilinear(frame_size, focal_length, 0.0);
        }

        /**
         * Estimates the magnification based on focal length and focus distance.
         * @param focal_length The focal length
         * @param focus_distance The focus distance
         * @return The estimated magnification
         */
        template <typename T>
        double maginifaction(T focal_length, T focus_distance) {
            static_assert(std::is_floating_point<T>::value, "Wrong type.");
            return focal_length / (focus_distance - focal_length);
        }

    }

    /**
     * \brief Indicated the direction of a slobe
     */
    enum class SlobeDirection {
        VERTICAL,
        HORIZONTAL,
        DESCENDING,
        ASCENDING
    };

    /** Brief Calculates the manhattan distance
    * Manhattan distance between two points
    * @param x1 Point #1 x
    * @param x2 Point #2 x
    * @param y1 Point #1 y
    * @param y2 Point #2 y
    * @return The manhattan distance between the two points
    */
    template <typename T>
    T dist_manhattan(const T x1, const T x2, const T y1, const T y2) {
        static_assert(std::is_arithmetic<T>::value, "dist_manhattan is only possible for arithmetic types.");
        return abs(x2 - x1 + y2 - y1);
    }

    /**
     * \brief Computes the distance (pyta)
     * \tparam T The type indicator
     * \param x1 X of first point
     * \param x2 X of second point
     * \param y1 Y of first point
     * \param y2 Y of second point
     * \return The distance (rooted)
     */
    template <typename T>
    double dist_real(const T x1, const T x2, const T y1, const T y2) {
        static_assert(std::is_arithmetic<T>::value, "dist_real is only possible for arithmetic types.");
        double x = pow(x2 - x1, 2);
        double y = pow(y2 - y1, 2);
        return sqrt(x + y);
    }

    /**
     * \brief Converts radians to degrees
     * \tparam T The type, only floating point allowed
     * \param radians The radians to be converted
     * \return The angle in degrees
     */
    template <typename T>
    double rad_to_deg(T radians) {
        static_assert(std::is_floating_point<T>::value, "rad_to_deg is only possible for floating point.");
        return (radians * 180.0) / PI;
    }

    /**
     * \brief Converts degrees to radians
     * \tparam T The type, only floating point allowed
     * \param degrees The degrees to convert
     * \return The degrees in radians
     */
    template <typename T>
    double deg_to_rad(T degrees) {
        static_assert(std::is_floating_point<T>::value, "deg_to_rad is only possible for floating point.");
        return (degrees * PI) / 180;
    }

    /**
     * \brief Determin the angle between two points in radians
     * \tparam T Typename
     * \param x1 X of first point
     * \param x2 X of second point
     * \param y1 Y of first point
     * \param y2 Y of second point;
     * \return The angle in degrees
     */
    template <typename T>
    double angle_between_points(const T x1, const T x2, const T y1, const T y2) {
        static_assert(std::is_arithmetic<T>::value, "angle_between_points is only possible for arithmetic types.");
#ifdef CV_VERSION
        return cv::fastAtan2(y2 - y1, x2 - x1);
#else
		return atan2(y2 - y1, x2 - x1);
#endif
    }

    /**
     * \brief Calculates the slobe between two points
     * \tparam T1 The type of point 1
     * \tparam T2 The type of point 2
     * \param x1 X of point 1
     * \param x2 X of point 2
     * \param y1 Y of point 1
     * \param y2 Y of point 2
     * \return The slobe
     */
    template <typename T1, typename T2>
    double slope(const T1 x1, const T2 x2, const T1 y1, const T2 y2) {
        static_assert(std::is_arithmetic<T1>::value || std::is_arithmetic<T2>::value, "slope is only possible for arithmetic types.");
        static_assert(std::is_convertible<T1, T2>::value, "slope argument types must be convertible.");
        double dx = x2 - x1;
        if (dx == 0.0)
            return 0.0;
        double dy = y2 - y1;
        return dy / dx;
    }

    /**
     * \brief Determine slobe direction
     * \param slope The slobe to determin
     * \return Enum for slobe direction
     */
    template <typename T>
    SlobeDirection slobe_direction(T slope) {
        static_assert(std::is_arithmetic<T>::value, "slobe_direction is only possible for arithmetic types.");
        if (std::isinf(slope))
            return SlobeDirection::VERTICAL;
        if (slope < 0.0)
            return SlobeDirection::DESCENDING;
        if (slope > 0.0)
            return SlobeDirection::ASCENDING;
        return SlobeDirection::HORIZONTAL;
    }

    /**
     * \brief Angle between lines
     * \tparam T Type
     * \param x1 X of point 1
     * \param x2 X of point 2
     * \param y1 Y of point 1
     * \param y2 Y of point 2
     * \return 
     */
    template <typename T>
    double angle_between_lines(T x1, T x2, T y1, T y2) {
        static_assert(std::is_arithmetic<T>::value, "angle_between_lines is only possible for arithmetic types.");
        return atan(static_cast<double>(y1 - y2) / static_cast<double>(x2 - x1));
    }

    /**
     * \brief Calculates the angle between two points with a specified central point
     * NOT WORKING ATM!
     * \tparam T The type
     * \param p1_x X of point 1
     * \param p2_x X of point 2
     * \param c_x X of central point
     * \param p1_y Y of point 1
     * \param p2_y Y of point 2
     * \param c_y Y of central point
     * \return The angle in radians
     */
    template <typename T>
    double angle_inner_points(const T p1_x, const T p2_x, const T c_x, const T p1_y, const T p2_y, const T c_y) {

        // this function seems to be WRONG!.. (somebody shoot me!)

        static_assert(std::is_arithmetic<T>::value, "angle_inner_points is only possible for arithmetic types.");
        auto dist1 = cv::sqrt((p1_x - c_x) * (p1_x - c_x) + (p1_y - c_y) * (p1_y - c_y));
        auto dist2 = cv::sqrt((p2_x - c_x) * (p2_x - c_x) + (p2_y - c_y) * (p2_y - c_y));

        double ax, ay;
        double bx, by;

        auto cx = c_x;
        auto cy = c_y;

        // checking for closest point to c
        if (dist1 < dist2) {
            bx = p1_x;
            by = p1_y;
            ax = p2_x;
            ay = p2_y;
        } else {
            bx = p2_x;
            by = p2_y;
            ax = p1_x;
            ay = p1_y;
        }

        auto q_1 = cx - ax;
        auto q_2 = cy - ay;
        auto p_1 = bx - ax;
        auto p_2 = by - ay;

        auto a = acos((p_1 * q_1 + p_2 * q_2) / (std::sqrt(p_1 * p_1 + p_2 * p_2) * std::sqrt(q_1 * q_1 + q_2 * q_2)));

        return a;
    }

    /**
     * \brief Solves a quadratic equation
     * \tparam T The base type
     * \param a a operand
     * \param b b operand
     * \param c c operanc
     * \return Tuple in format <bool, T, T>, where bool indicates if both roots are real regardless if they are the same or not.
     */
    template <typename T>
    std::tuple<bool, double, double> quadratic_equation(T a, T b, T c) {
        static_assert(std::is_arithmetic<T>::value, "quadratic_equation is only possible for arithmetic types.");

        double a2 = a * a;
        double det = b * b - 2 * a2 * c;

        if (det > 0.0) {
            det = std::sqrt(det);
            return std::make_tuple(true, (-b + det) / a2, (-b - det) / a2);
        }

        if (det == 0.0) {
            det = std::sqrt(det);
            double res = (-b + det) / a2;
            return std::make_tuple(true, res, res);
        }

        return std::make_tuple(true, -b / a2, std::sqrt(-det) / a2);

    }

    /**
     * \brief Computes mu
     * Simple mu computation helper function for use with Interpolate and Bezier calculations
     * \param current_pos The current position
     * \param in_between How many points there are in the source position and the target position
     * \return The computed mu
     */
    template <typename T>
    double mu(T current_pos, T in_between) {
        static_assert(std::is_arithmetic<T>::value, "mu is only possible for arithmetic types.");
        return current_pos / in_between;
    }

#ifdef CV_VERSION

    /**
     * @overload For opencv point types
     * \tparam T1 Type of point one 
     * \tparam T2 Type of point two
     * \param p1 The first point
     * \param p2 The second point
     * \return The manhattan distance between the two points
     */
    template <typename T1, typename T2>
    double dist_manhattan(cv::Point_<T1>& p1, cv::Point_<T2>& p2) {
        static_assert(std::is_arithmetic<T1>::value, "dist_manhattan T1 is only possible for arithmetic types.");
        static_assert(std::is_arithmetic<T2>::value, "dist_manhattan T2 is only possible for arithmetic types.");
        return dist_manhattan(p1.x, p2.x, p1.y, p1.y);
    }

    /**
     * \brief OpenCV normalizations function wrapper
     * \tparam T1 Type for point 1
     * \tparam T2 Type for point 2
     * \param p1 The first point
     * \param p2 The second point
     * \return The normalized dist
     */
    template <typename T1, typename T2>
    double dist_norm(cv::Point_<T1>& p1, cv::Point_<T2>& p2) {
        static_assert(std::is_arithmetic<T1>::value || std::is_arithmetic<T2>::value, "dist_real is only possible for arithmetic types.");
        return cv::norm(p2 - p1);
    }

    /**
     * @overload
     * \tparam T1 Type of point 1
     * \tparam T2 Type of point 2
     * \param p1 The first point
     * \param p2 The second point
     * \return The slobe as double float
     */
    template <typename T1, typename T2>
    double slope(cv::Point_<T1>& p1, cv::Point_<T2>& p2) {
        static_assert(std::is_arithmetic<T1>::value || std::is_arithmetic<T2>::value, "slope is only possible for arithmetic types.");
        return slope(p1.x, p2.x, p1.y, p2.y);
    }

    /**
     * \brief Computes the angle between two points (from OREGO)
     * \tparam T Type of vector
     * \param v1 The vector containing the two points
     * \return The angle in RADIANS
     */
    template <typename T>
    double angle(cv::Vec<T, 4>& v1) {
        static_assert(std::is_arithmetic<T>::value, "Wrong type.");

        double len_1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
        double len_2 = sqrt(v1[2] * v1[2] + v1[3] * v1[3]);

        double dot = v1[0] * v1[2];
        dot += v1[1] * v1[3];

        auto a = dot / (len_1 * len_2);

        if (a >= 1.0)
            return 0.0;

        if (a <= -1.0)
            return PI;

        return acos(a); // 0..PI
    }

    /**
     * \brief Computes the angle between two points (from OREGO)
     * \tparam T Type of points
     * \param p1 The first point
     * \param p2 The second point
     * \return The angle between the points in RADIANS
     */
    template <typename T>
    double angle(cv::Point_<T>& p1, cv::Point_<T>& p2) {
        return angle(cv::Vec<T, 4>(p1.x, p1.y, p2.x, p2.y));
    }

    /**
     * @overload
     * \tparam T1 Type of line point one
     * \tparam T2 Type of line point two
     * \param p1 Line point 1
     * \param p2 Line point 2
     * \return The angle in radians
     */
    template <typename T1, typename T2>
    double angle_between_lines(cv::Point_<T1>& p1, cv::Point_<T2>& p2) {
        static_assert(std::is_arithmetic<T1>::value || std::is_arithmetic<T2>::value, "angle_between_lines is only possible for arithmetic types.");
        return atan((p1.y - p2.y) / (p2.x - p1.x));
    }

    /**
     * \brief Calculates the angle of a 2D vector in degrees.
     * Invokes the OpenCV function fastAtan2 which calculates the full-range angle of an input 2D vector.
     * \tparam T The type, only floating points.
     * \param x The x-coordinate
     * \param y The y-coordinate
     * \return The angle in radians
     */
    template <typename T>
    double angle_from_zero(T x, T y) {
        static_assert(std::is_floating_point<T>::value, "only possible with floating points.");
        return deg_to_rad(cv::fastAtan2(x, y));
    }

    /**
     * @overload
     * \tparam T The type
     * \param point The point with x- and y-coordinates
     * \return The angle in degrees
     */
    template <typename T>
    double angle_from_zero(cv::Point_<T>& point) {
        static_assert(std::is_floating_point<T>::value, "only possible with floating points");
        return angle_from_zero(point.x, point.y);
    }

    /**
    * \brief Computes the inner angle between two points both intersection with a center point
    * \param p1 The first point
    * \param p2 The second point
    * \param c The center point
    * \return The angle in degrees
    */
    template <typename T>
    double angle_inner_points(const cv::Point_<T>& p1, const cv::Point_<T>& p2, const cv::Point_<T>& c) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        return angle_inner_points(p1.x, p2.x, c.x, p1.y, p2.y, c.y);
    }

    /**
     * \brief Simple function wrapping for checking if two angles are within a specific margin of each other
     * \param angle_a The first angle
     * \param angle_b The second angle
     * \param acceptance_margin The margin that is allowed (defaults to 0.5)
     * \return true if the angles are withing the specified margin, otherwise false
     */
    inline bool is_withing_margin(double angle_a, double angle_b, double acceptance_margin = 0.5) {
        auto angle_deg = abs(rad_to_deg(angle_a) - rad_to_deg(angle_b));
        return angle_deg <= acceptance_margin;
    }

    /**
    * \brief Computes intersection points based on 4 points (2 x 2)
    * \param o1 Point one of pair one
    * \param p1 Point one of pair two
    * \param o2 Point two of pair one
    * \param p2 Point two of pair two
    * \param result The resulting point of intersection
    * \return true if intersection was found, otherwise false
    */
    template <typename T>
    bool intersection(cv::Point_<T> o1, cv::Point_<T> p1, cv::Point_<T> o2, cv::Point_<T> p2, cv::Point_<T>& result) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        auto d1 = p1 - o1;
        auto d2 = p2 - o2;

        auto cross = d1.x * d2.y - d1.y * d2.x;
        if (abs(cross) < /*EPS*/1e-8)
            return false;

        auto x = o2 - o1;

        auto t1 = (x.x * d2.y - x.y * d2.x) * (1.0 / cross);
        result = o1 + d1 * t1;
        return true;
    }

    /** @overload
    * \param o1 Point one of pair one
    * \param p1 Point one of pair two
    * \param o2 Point two of pair one
    * \param p2 Point two of pair two
    * \return true if intersection was found, otherwise false
    */
    template <typename T>
    bool intersection(cv::Point_<T> o1, cv::Point_<T> p1, cv::Point_<T> o2, cv::Point_<T> p2) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        auto d1 = p1 - o1;
        auto d2 = p2 - o2;

        auto cross = d1.x * d2.y - d1.y * d2.x;

        // check for EPS boundry
        if (abs(cross) < 1e-8)
            return false;
        return true;
    }

    /** @overload
    * \param border The border vector containing x/y coordinates for both borders
    * \param line The line vector The line for which to check intersections with borders
    * \param result The resulting point coordinates if they intersect
    * \return true if intersection was found, otherwise false
    */
    template <typename T>
    bool intersection(const cv::Vec<T, 4>& border, cv::Vec<T, 4>& line, cv::Point_<T>& result) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        return intersection(cv::Point_<T>(border[0], border[1]), cv::Point_<T>(line[0], line[1]), cv::Point_<T>(border[2], border[3]), cv::Point_<T>(line[2], line[3]), result);
    }

    /** @overload
    * \param border The border vector containing x/y coordinates for both borders
    * \param line The line vector The line for which to check intersections with borders
    * \return true if intersection was found, otherwise false
    */
    template <typename T>
    bool intersection(const cv::Vec<T, 4>& border, cv::Vec<T, 4>& line) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        return intersection(cv::Point_<T>(border[0], border[1]), cv::Point_<T>(line[0], line[1]), cv::Point_<T>(border[2], border[3]), cv::Point_<T>(line[2], line[3]));
    }

    /**
    * \brief Computes the intersection points based on horizontal line and two borders
    * \tparam T 
    * \param horizontal_line The horizontal line
    * \param left_border The left border
    * \param right_border The right border
    * \param output The resulting intersection points (if any)
    * \return true if intersection points where computed, otherwise false
    */
    bool compute_intersection_points(cv::Vec4d& horizontal_line, const cv::Vec4d& left_border, const cv::Vec4d& right_border, cv::Vec4d& output);

    /**
     * \brief Computes the margin of offset from the borders. This is depending on (for now unspecified, set to 40) curcomstances
     * for which the close are near the borders should NOT be considered.
     * \tparam T Type of vectors
     * \param left_border The left border area
     * \param right_border The right border area
     * \return The newly computed margin
     */
    template <typename T>
    cv::Vec<T, 2> compute_intersection_cut(const cv::Vec<T, 4>& left_border, const cv::Vec<T, 4>& right_border) {

        // TODO : do something here

        return cv::Vec<T, 2>(40.0, 40.0);
    }

    /**
     * \brief Computes the average of values in a vector, wrapped in safety
     * \tparam T The type of value
     * \param vec The vector containing the values
     * \return The average as double precision floating point
     */
    template <typename T>
    double avg(std::vector<T>& vec) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");

        auto sum = 0.0;
        if (vec.empty())
            return sum;

        if (vec.size() == 1)
            return vec.front();

        for (const auto v : vec)
            sum += v;

        return sum / static_cast<double>(vec.size());
    }

    /**
     * \brief Computes a avg rectangle
     * \tparam T The type of input rectangle
     * \param vec_rects The vector containing the rectangles to compute avg of
     * \return Rectangle (double type) with the avg values
     */
    template <typename T>
    cv::Rect2d avg(std::vector<cv::Rect_<T>>& vec_rects) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");

        if (vec_rects.empty())
            return cv::Rect_<T>(0, 0, 0, 0);

        if (vec_rects.size() == 1)
            return cv::Rect_<T>(vec_rects.front());

        cv::Rect2d tmp;

        for (const auto& r : vec_rects) {
            tmp.x += r.x;
            tmp.y += r.y;
            tmp.width += r.width;
            tmp.height += r.height;
        }

        tmp.x /= vec_rects.size();
        tmp.y /= vec_rects.size();
        tmp.width /= vec_rects.size();
        tmp.height /= vec_rects.size();

        return cv::Rect2d(tmp);
    }

    /**
     * \brief Calculates the avg of all the Y values in the vector of points
     * \tparam T The type of the points
     * \param vec The vector containing the points
     * \return 0.0 if vector is empty, else the avg of the Y values for all the points
     */
    template <typename T>
    double avg_y(std::vector<cv::Point_<T>>& vec) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");

        auto sum = 0.0;

        if (vec.empty())
            return sum;

        for (const auto& v : vec)
            sum += v.y;

        return sum / static_cast<double>(vec.size());
    }

    /**
     * \brief Computes the average Y values of a vector containing rectangle types
     * \tparam T The type of rectangle
     * \param vec_rects The vector containing the rectangles
     * \return The avg value of all Y values
     */
    template <typename T>
    double avg_y(std::vector<cv::Rect_<T>>& vec_rects) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");

        auto sum = 0.0;

        if (vec_rects.empty())
            return sum;

        if (vec_rects.size() == 1)
            return vec_rects.front().y;

        for (const auto& r : vec_rects)
            sum += r.y;

        return sum / static_cast<double>(vec_rects.size());
    }

    /**
     * \brief Calculates the avg of all the X values in the vector of points
     * \tparam T The type of the points
     * \param vec The vector containing the points
     * \return 0.0 if vector is empty, else the avg of the X values for all the points
     */
    template <typename T, int cn>
    double avg_y(cv::Vec<T, cn>& vec) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        static_assert(cn == 4, "avg_y only supports Vec<T, 4>");
        return (vec[1] + vec[3]) / 2.0;
    }

    /**
     * \brief Computes the average Y value of parsed 6-length vector
     * \tparam T Type of vector
     * \param vec The vector to compute average Y values from
     * \return The average Y value
     */
    template <typename T>
    double avg_y(cv::Vec<T, 6>& vec) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        return (vec[1] + vec[3] + vec[5]) * (1.0 / 3.0);
    }

    /**
     * \brief Computes the average X values of vectors within parsed vector
     * \tparam T The type
     * \param vec The vector containing the vectors to compute average X from
     * \return The average of all vector X values
     */
    template <typename T>
    double avg_x(std::vector<cv::Point_<T>>& vec) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");

        auto sum = 0.0;

        if (vec.empty())
            return sum;

        for (const auto& v : vec)
            sum += v.x;

        return sum / vec.size();
    }

    /**
     * \brief Computes the avg X of a 4 sized vector of arbitrary type
     * \tparam T Type of vector
     * \param vec The vector (4 sized) to get the X avg from
     * \return The average X as double precision floating point
     */
    template <typename T>
    double avg_x(cv::Vec<T, 4>& vec) {
        static_assert(std::is_arithmetic<T>::value, "avg_x is only possible for arithmetic types.");
        return (vec[0] + vec[2]) / 2;
    }

    /**
     * \brief Computes the avg X of a 6 sized vector of arbitrary type
     * \tparam T The type
     * \param vec The vector
     * \return The average of X as double precision floating point
     */
    template <typename T>
    double avg_x(cv::Vec<T, 6>& vec) {
        static_assert(std::is_arithmetic<T>::value, "avg_x is only possible for arithmetic types.");
        return (vec[0] + vec[2] + vec[4]) * (1 / 3);
    }

    /**
     * \brief Computes the avg X of an arbitraty vector
     * \tparam T The type of the vector
     * \tparam C The size of the vector, must be greater than zero
     * \param vec The vector to compute X avg of
     * \return The avg of X values as double precision floating point
     */
    template <typename T, int C>
    double avg_x(cv::Vec<T, C>& vec) {
        static_assert(std::is_arithmetic<T>::value, "avg_x is only possible for arithmetic types.");
        static_assert(C > 0, "Size must be greater than zero.");
        auto sum = 0.0;
        for (auto i = 0; i < C; i += 2)
            sum += vec[i];
        return sum * (1 / (C / 2));
    }

    /**
     * \brief Computes the avg of both X and Y values of a vector
     * \tparam T The type of the vector
     * \tparam C The size of the vector, must be greater than zero
     * \param vec The vector to compute the avg of
     * \return A single vector of size 2 with the average values of the same type as input vector
     */
    template <typename T, int C>
    cv::Vec<T, 2> avg_xy(cv::Vec<T, C>& vec) {
        static_assert(std::is_floating_point<T>::value, "avg_xy is only possible for floats.");
        static_assert(C > 0, "Size must be greater than zero.");
        auto sumX = 0.0;
        auto sumY = 0.0;
        for (auto i = 0; i < C; i += 2) {
            sumX += vec[i];
            sumY += vec[i + 1];
        }
        return cv::Vec<T, C>(sumX, sumY);
    }

    /**
     * \brief Compute the avg X and Y of a vector of points
     * \tparam T The type of points
     * \param vec The vector containing the points
     * \return A single vector of double precision floating point which holds the average X and Y values
     */
    template <typename T>
    cv::Vec2d avg_xy(std::vector<cv::Point_<T>>& vec) {
        static_assert(std::is_arithmetic<T>::value, "avg_xy is only possible for arithmetic types.");

        cv::Vec2d sum(0.0, 0.0);

        if (vec.empty())
            return sum;

        for (const auto& v : vec) {
            sum[0] += v.x;
            sum[1] += v.y;
        }

        sum[0] /= vec.size();
        sum[1] /= vec.size();

        return cv::Vec2d(sum);
    }

    /**
     * \brief Generates a simple random number (pseudo)
     * \tparam T Type of value to produce
     * \param min The min value
     * \param max The max value
     * \return Random number between min and max
     */
    template <typename T>
    T rngeezuz(T min, T max) {
        auto f = static_cast<T>(rand()) / RAND_MAX;
        return min + f * (max - min);
    }

    /**
     * \brief Computes the intensity centroid for each X in the Y direction.
     * This gives the weighted Y position based off the intensity levels across that single X column
     * \param image The image to perform the computation on
     * \param output The output vector of points
     * \param upper_limit The upper limit of the rectangular cut out
     * \param lower_limit The lower limit of the rectangular cut out
     * \return The avg of the computed Y value across the entirety of the image matrix with regards to cut offs
     */
    template <typename T>
    double real_intensity_line(cv::Mat& image, std::vector<cv::Point_<T>>& output, int upper_limit, int lower_limit) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");

        /*
         * NEVER CHANGE ANYTHING IN HERE
         * ---> WORK ON CHANGES IN A COPY OF IT!
         */

        // grab sizes
        auto cols = image.cols;
        //auto rows = image.rows; // not used for anything atm

        // cut out rectangle without any X value set
        cv::Rect_<T> cut_rect(0.0, lower_limit, 1.0, upper_limit);

        // populate the output vector with column based X values and zero Y
        stl::populate_x(output, cols);

        for (auto& v : output) {
            // adjust cutOut rectangle for current position
            cut_rect.x = v.x;

            // create a new matrix based of the settings 
            auto B = cv::Mat(image, cut_rect);

            // perform moments on the matrix without treating it as a binary image (which it is NOT)
            auto m = cv::moments(B, false);

            // compute x & y
            //auto x = m.m10 / m.m00; // x is not used, so no need to calculate it
            auto y = m.m01 / m.m00;

            // only include values above 0.0 in y-pos
            if (y > 0.0)
                v.y = y;
        }

        return avg_y(output);
    }

    /**
     * \brief Computes the real intensity line (condensed version).
     * Each column is sliced and the weigthed mass of the intensity levels are computed
     * to a single Y value for each X value.
     * \tparam T The type The type of points to put the line into
     * \param image The image to be sliced
     * \param output The resulting points
     * \return The avg value of the entirety of the resulting new Y values
     */
    template <typename T>
    double real_intensity_line(cv::Mat& image, std::vector<cv::Point_<T>>& output) {

        auto cols = image.cols;

        cv::Rect_<T> cut_rect(0.0, 0.0, 1.0, image.rows);

        stl::populate_x(output, cols);

        for (auto& v : output) {

            cut_rect.x = v.x;

            auto B = cv::Mat(image, cut_rect);

            auto m = cv::moments(B, false);

            auto y = m.m01 / m.m00;

            if (y > 0.0)
                v.y = y;

        }

        auto avg = avg_y(output);

        if (avg == 0.0) {
            log_err << __FUNCTION__ << " got 0 avg value.\n";
        }

        return avg;

    }

    /**
     * \brief Computes the avg mass weigth position based on intensity
     * \tparam T Point type
     * \param image The image, a good idea to narrow it for the area in question
     * \param target_vector Will be populated with 0->N of X and weighted mass of intensity in Y for each column (x)
     * \return The avg of the whole result based on the values in the target vector
     */
    template <typename T>
    double weighted_avg(cv::Mat& original, cv::Mat& image, std::vector<cv::Point_<T>>& target_vector, cv::Rect& laser_rect_out) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        std::vector<cv::Point> non_zero_elements(image.rows * image.cols);
        findNonZero(image, non_zero_elements);
        laser_rect_out = boundingRect(non_zero_elements);
        // the argument for "original" image is stricly not required as is.
        // but does allow for easy future expansion and modifications
        auto t = original(laser_rect_out);
        return real_intensity_line(t, target_vector, t.rows, 0);
    }

#else

    /* shadow versions of most functions for custom class instead of points (might not be complete!!!!!) */

	template <typename T>
	inline
	double dist_manhattan(v2<T>& p1, v2<T>& p2) {
		static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
		return dist_manhattan(p1.x, p2.x, p1.y, p1.y);
	}

	template <typename T>
	inline
	double dist_real(v2<T>& p1, v2<T>& p2) {
		static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
		return dist_real(p1.x, p2.x, p1.y, p2.y);
	}

	template <typename T>
	inline
	double slope(v2<T>& p1, v2<T>& p2) {
		static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
		return slope(p1.x, p2.x, p1.y, p2.y);
	}

	template <typename T>
	inline
	double angle_between_lines(v2<T>& p1, v2<T>& p2) {
		static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
		return atan((p1.y - p2.y) / (p2.x - p1.x) * calc::DEGREES);
	}

	template <typename T>
	inline
	double angle_inner_points(const v2<T>& p1, const v2<T>& p2, const v2<T>& c) {
		static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
		return angle_inner_points(p1.x, p2.x, c.x, p1.y, p2.y, c.y);
	}

    /**
    * \brief Computes intersection points based on 4 points (2 x 2)
    * \param o1 Point one of pair one
    * \param p1 Point one of pair two
    * \param o2 Point two of pair one
    * \param p2 Point two of pair two
    * \param result The resulting point of intersection
    * \return true if intersection was found, otherwise false
    */
	template <typename T>
	inline
	bool intersection(v2<T> o1, v2<T> p1, v2<T> o2, v2<T> p2, v2<T>& result) {
		static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");

		auto d1 = p1 - o1;
		auto d2 = p2 - o2;

		auto cross = d1.x * d2.y - d1.y * d2.x;
		if (abs(cross) < /*EPS*/1e-8)
			return false;

		auto x = o2 - o1;

		auto t1 = (x.x * d2.y - x.y * d2.x) / cross;
		result = o1 + d1 * t1;
		return true;
	}


#endif

    /** Brief Determines the highest of two values
    * Max of two ints
    * @param a operand #1
    * @param b operand #2
    * @return the highest of the two operands, defaults to operand #1
    */
    template <typename T>
    T maxval(T a, T b) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        return a ^ ((a ^ b) & -(a < b)); // max(x, y)
        //return max(a, b);
    }

    /** Brief Determines the highest of three values
    * Max of three arithmetic values
    * @param a operand #1
    * @param b operand #2
    * @param c operand #3
    * @return the highest of the three operands
    */
    template <typename T>
    T maxval(T a, T b, T c) {
        static_assert(std::is_arithmetic<T>::value, "type is only possible for arithmetic types.");
        return maxval(maxval(a, b), c);
    }

    /**
     * \brief Branchless min() function for integrals
     * \tparam T The type
     * \param a The first value
     * \param b The second value
     * \return The lowest of the two values
     */
    template <typename T>
    double minval(T a, T b) {
        static_assert(std::is_integral<T>::value, "Wrong type.");
        return b ^ ((a ^ b) & -(a < b)); // min(x, y)
        //return a < b ? a : b;
    }

    /**
     * \brief Computes the minimum value of three values
     * \tparam T1 The type of the first value
     * \tparam T2 The type of the second value
     * \tparam T3 The type of the third value
     * \param a The first value
     * \param b The second value
     * \param c The third value
     * \return The lowest of the three values
     */
    template <typename T1, typename T2, typename T3>
    double minval(T1 a, T2 b, T3 c) {
        static_assert(std::is_integral<T1>::value && std::is_integral<T2>::value && std::is_integral<T3>::value, "Incompatible types.");
        return minval(minval(a, b), c);
    }

    /**
     * \brief Stupid fast value in-between check
     * \tparam T Value type of boundries
     * \param value The value to check
     * \param lower The lower boundry
     * \param upper The upper boundry
     * \return true if value is in between lower and upper
     */
    template <typename T>
    bool in_between(int value, T lower, T upper) {
        static_assert(std::is_integral<T>::value, "invalid type.");
        return (static_cast<unsigned int>(value) - static_cast<unsigned int>(lower) < static_cast<unsigned int>(upper) - static_cast<unsigned int>(lower));
    }

    /**
     * \brief Computer the variance coefficient
     * \tparam T1 Type for s
     * \tparam T2 Type for mean
     * \param s s
     * \param mean mean
     * \return the variance coefficnent
     */
    template <typename T1, typename T2>
    constexpr double variance_coeff(T1 s, T2 mean) {
        static_assert(std::is_same<T1, double>::value, "s must be double floating point.");
        static_assert(std::is_convertible<T1, T2>::value, "s and mean must be convertible.");
        return s / mean * 100.0;
    }

    /**
     * \brief Determine if the parsed value is a power of two or not (v^2)
     * \tparam T The type, only unsigned allowed
     * \param v The value to check
     * \return true if the value is a power of 2, otherwise false
     */
    template <typename T>
    constexpr bool is_pow_2(T v) noexcept {
        statis_assert(std::is_unsigned<T>::value, "Wrong type.");
        return v && !(v & (v - 1));
    }

    /**
     * \brief Directly swaps two values in place.
     * \tparam T Type of values to swap
     * \param first The first value
     * \param second The second value
     */
    template <typename T>
    void swap(T& first, T& second) noexcept {
        static_assert(std::is_integral<T>::value, "Wrong type.");
        first ^= second;
        second ^= first;
        first ^= second;
    }

    /**
     * \brief Swaps N amount of bits at specified locations
     * \tparam T Type of how many bits to swap
     * \param first_pos The first position
     * \param second_pos The second position
     * \param bit_amount The amount of bits to swap
     * \param bits The word that contains the bits to be swapped
     * \return Newly created word where the bits are swapped
     */
    template <typename T>
    uint_fast64_t swap_bits(uint_fast64_t first_pos, uint_fast64_t second_pos, T bit_amount, uint_fast64_t bits) {
        static_assert(std::is_unsigned<T>::value, "Wrong type.");
        unsigned long x = ((bits >> first_pos) ^ (bits >> second_pos)) & ((1U << bit_amount) - 1); // XOR temporary
        return bits ^ ((x << first_pos) | (x << second_pos));
    }

    /**
     * \brief Computes the absolute value (DONT ACTIVATE, MSC DONT LIKE IT!)
     * \tparam T Type of value, must be integral
     * \param v The value
     * \return The absolute value of v
     */
    //template <typename T>
    //T abs(T v) {
    //    static_assert(std::is_integral<T>::value, "Wrong type.");
    //    int const mask = v >> sizeof(int) * (CHAR_BIT - 1);
    //    return (v ^ mask) - mask;
    //}

    /**
     * \brief Sets or clears bits from a mask in a given word.
     * \tparam T Type of mask
     * \param condition if true, the bit(s) are set, otherwise cleared
     * \param mask The mask to set or clear
     * \param word The word where the bits should be set or cleared
     * \return Newly formed word with indicated bit either set or cleared in
     */
    template <typename T>
    uint_fast64_t set_or_clear(const bool condition, T mask, uint_fast64_t word) {
        static_assert(std::is_integral<T>::value, "Wrong type.");
        return word ^= (-condition ^ word) & mask;
    }

    /**
     * \brief Negates an integral type
     * \tparam T The type of value, must be integral
     * \param value The value to negate
     * \param condition If true, value is negated, otherwise it's not
     * \return The value negated or not depending on condition
     */
    template <typename T>
    T negate(T value, bool condition) {
        static_assert(std::is_integral<T>::value, "Wrong type.");
        return (value ^ -condition) + condition;
    }

    /**
     * \brief Counts the number of bits set in a given bit word
     * \tparam T Type of bit word
     * \param v The bit word to count
     * \return The number of bits counted
     */
    template <typename T>
    int count_bits(T v) {
        static_assert(std::is_integral<T>::value, "Wrong type.");
        v = v - ((v >> 1) & static_cast<T>(~static_cast<T>(0)) / 3); // temp
        v = (v & static_cast<T>(~static_cast<T>(0)) / 15 * 3) + ((v >> 2) & static_cast<T>(~static_cast<T>(0)) / 15 * 3); // temp
        v = (v + (v >> 4)) & static_cast<T>(~static_cast<T>(0)) / 255 * 15; // temp
        return static_cast<T>(v * (static_cast<T>(~static_cast<T>(0)) / 255)) >> (sizeof(T) - 1) * CHAR_BIT; // count
    }

}
