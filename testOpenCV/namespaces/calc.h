#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/videostab/inpainting.hpp>

#include "tg.h"
#include "stl.h"

#ifndef CV_VERSION
#include "Util/Vec.h"
#endif

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#elif defined(__GNUC__) && !defined(inline)
#define inline __always_inline
#endif

/**
 * \brief Calculation utility functionality
 * Contains optional OpenCV overloads for quick access
 */
namespace calc {

#define C_PI   3.1415926535897932384626433832795
#define C_2PI  6.283185307179586476925286766559
#define C_LOG2 0.69314718055994530941723212145818

	/**
	 * \brief Indicated the direction of a slobe
	 */
	enum class SlobeDirection {
		VERTICAL,
		HORIZONTAL,
		DESCENDING,
		ASCENDING
	};

	/**
	* Round to the nearest integer (stolen from opencv)
	* @param value The value to round
	* @return Nearest integer as double
	*/
	inline
	int round(double value) {
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
		return static_cast<int>(floor(d + 0.5));
#endif
	}

	/** Brief Calculates the manhattan distance
	* Manhattan distance between two points
	* @param x1 Point #1 x
	* @param x2 Point #2 x
	* @param y1 Point #1 y
	* @param y2 Point #2 y
	* @return The manhattan distance between the two points
	*/
	template <typename T>
	inline
	T dist_manhattan(const T x1, const T x2, const T y1, const T y2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
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
	inline
	double dist_real(const T x1, const T x2, const T y1, const T y2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		T x = pow(x2 - x1, 2);
		T y = pow(y2 - y1, 2);
		return sqrt(x + y);
	}

	inline
	double rad_to_deg(double radians) {
		return radians * 180 / C_PI;
	}

	/**
	 * \brief Determin the angle between two points in radians
	 * \tparam T Typename
	 * \param x1 X of first point
	 * \param x2 X of second point
	 * \param y1 Y of first point
	 * \param y2 Y of second point;
	 * \return The angle in radians
	 */
	template <typename T>
	inline
	double angle_between_points(const T x1, const T x2, const T y1, const T y2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
#ifdef CV_VERSION
		return cv::fastAtan2(y2 - y1, x2 - x1);
#else
		return atan2(y2 - y1, x2 - x1);
#endif
	}

	template <typename T>
	inline
	double slope(const T x1, const T x2, const T y1, const T y2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		double dx = x2 - x1;
		double dy = y2 - y1;
		return dy / dx;
	}

	inline
	SlobeDirection slobe_direction(const double slope) {
		if (std::isinf(slope))
			return SlobeDirection::VERTICAL;
		if (slope < 0.0)
			return SlobeDirection::DESCENDING;
		if (slope > 0.0)
			return SlobeDirection::ASCENDING;
		return SlobeDirection::HORIZONTAL;
	}

	/**
	 * \brief 
	 * \tparam T 
	 * \param x1 
	 * \param x2 
	 * \param y1 
	 * \param y2 
	 * \return 
	 */
	template <typename T>
	inline
	double angle_between_lines(T x1, T x2, T y1, T y2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return atan((y1 - y2) / (x2 - x1));
	}

	template <typename T>
	inline
	double angle_inner_points(const T p1_x, const T p2_x, const T c_x, const T p1_y, const T p2_y, const T c_y) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
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
		}
		else {
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

		a *= 180 / CV_PI;

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
	inline
	std::tuple<bool, double, double> quadratic_equation(T a, T b, T c) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

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
	inline
	double mu(T current_pos, T in_between) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return current_pos / in_between;
	}

#ifdef CV_VERSION
	template <typename T>
	inline
	double dist_manhattan(cv::Point_<T>& p1, cv::Point_<T>& p2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return dist_manhattan(p1.x, p2.x, p1.y, p1.y);
	}

	template <typename T>
	inline
	double dist_real(cv::Point_<T>& p1, cv::Point_<T>& p2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return cv::norm(p2 - p1);
	}

	template <typename T>
	inline
	double slope(cv::Point_<T>& p1, cv::Point_<T>& p2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return slope(p1.x, p2.x, p1.y, p2.y);
	}

	template <typename T>
	inline
	double angle_between_lines(cv::Point_<T>& p1, cv::Point_<T>& p2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return atan((p1.y - p2.y) / (p2.x - p1.x) * 180 / CV_PI);
	}

	/**
	* \brief Computes the inner angle between two points both intersection with a center point
	* \param p1 The first point
	* \param p2 The second point
	* \param c The center point
	* \return The angle in degrees
	*/
	template <typename T>
	inline
	double angle_inner_points(const cv::Point_<T>& p1, const cv::Point_<T>& p2, const cv::Point_<T>& c) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
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
	bool intersection(cv::Point_<T> o1, cv::Point_<T> p1, cv::Point_<T> o2, cv::Point_<T> p2, cv::Point_<T>& result) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
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

	/** @overload
	* \param border The border vector containing x/y coordinates for both borders
	* \param line The line vector The line for which to check intersections with borders
	* \param result The resulting point coordinates if they intersect
	* \return true if intersection was found, otherwise false
	*/
	template <typename T>
	inline
	bool intersection(const cv::Vec<T, 4>& border, cv::Vec<T, 4>& line, cv::Point_<T>& result) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return intersection(cv::Point_<T>(border[0], border[1]), cv::Point_<T>(line[0], line[1]), cv::Point_<T>(border[2], border[3]), cv::Point_<T>(line[2], line[3]), result);
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
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		// grab sizes
		auto cols = image.cols;
		//auto rows = image.rows; // not used for anything atm

		// cut out rectangle without any X value set
		cv::Rect_<T> cutOut(0.0, lower_limit, 1.0, upper_limit);

		// populate the output vector with column based X values and zero Y
		stl::populate_x(output, cols);

		for (auto& v : output) {
			// adjust cutOut rectangle for current position
			cutOut.x = v.x;

			// create a new matrix based of the settings 
			auto B = cv::Mat(image, cutOut);

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

	template <typename T>
	inline
	double weighted_avg(cv::Mat& image, std::vector<cv::Point_<T>>& target_vector) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		std::vector<cv::Point> non_zero_elements(image.rows * image.cols);
		findNonZero(image, non_zero_elements);
		auto laser_area = boundingRect(non_zero_elements);
		auto t = image(laser_area);
		return real_intensity_line(t, target_vector, t.rows, 0) + laser_area.y;
	}


	template <typename T>
	inline
	double avg_y(std::vector<cv::Point_<T>>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		auto sum = 0.0;

		if (vec.empty())
			return sum;

		for (const auto& v : vec)
			sum += v.y;

		return sum / vec.size();
	}

	template <typename T>
	inline
	double avg_y(cv::Vec<T, 4>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		return (vec[1] + vec[3]) / 2;
	}

	template <typename T>
	inline
	double avg_y(cv::Vec<T, 6>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		return (vec[1] + vec[3] + vec[5]) * (1 / 3);
	}

	template <typename T>
	inline
	double avg_x(std::vector<cv::Point_<T>>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		auto sum = 0.0;

		if (vec.empty())
			return sum;

		for (const auto& v : vec)
			sum += v.x;

		return sum / vec.size();
	}

	template <typename T>
	inline
	double avg_x(cv::Vec<T, 4>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		return (vec[0] + vec[2]) / 2;
	}

	template <typename T>
	inline
	double avg_x(cv::Vec<T, 6>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		return (vec[0] + vec[2] + vec[4]) * (1 / 3);
	}

	template <typename T>
	inline
	cv::Vec2d avg_xy(std::vector<cv::Point_<T>>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

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


#else

	template <typename T>
	inline
	double dist_manhattan(v2<T>& p1, v2<T>& p2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return dist_manhattan(p1.x, p2.x, p1.y, p1.y);
	}

	template <typename T>
	inline
	double dist_real(v2<T>& p1, v2<T>& p2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return dist_real(p1.x, p2.x, p1.y, p2.y);
	}

	template <typename T>
	inline
	double slope(v2<T>& p1, v2<T>& p2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return slope(p1.x, p2.x, p1.y, p2.y);
	}

	template <typename T>
	inline
	double angle_between_lines(v2<T>& p1, v2<T>& p2) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return atan((p1.y - p2.y) / (p2.x - p1.x) * 180 / CV_PI);
	}

	template <typename T>
	inline
	double angle_inner_points(const v2<T>& p1, const v2<T>& p2, const v2<T>& c) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
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
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

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
	template <class T>
	inline
	T maxval(T a, T b) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return max(a, b);
	}

	/** Brief Determines the highest of three values
	* Max of three fundamental values
	* @param a operand #1
	* @param b operand #2
	* @param c operand #3
	* @return the highest of the three operands
	*/
	template <class T>
	inline
	T maxval(T a, T b, T c) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		return maxval(maxval(a, b), c);
	}

}
