#pragma once

#include <vector>
#include "tg.h"

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#elif defined(__GNUC__) && !defined(inline)
#define inline __always_inline
#endif

namespace validate {

	/**
	 * \brief Validates a rectangle that its boundries are all valid numbers
	 * \tparam T The type of rectangle
	 * \param rect The rectangle to validate
	 * \return true if its values are above 0, otherweise false
	 */
	template <typename T>
	inline
	bool validate_rect(cv::Rect_<T>& rect) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		auto valid_rectangle = [rect]()-> bool {
			return rect.width > 0.0 && rect.height > 0.0 && rect.x >= 0.0 && rect.y >= 0.0;
		};

		return valid_rectangle();
	}

	/**
	 * \brief Validates a rectangle based on the boundries of an image
	 * \tparam T Type
	 * \param rect The rectangle to validate
	 * \param boundry The boundry matrix to compare the rectangle boundries to
	 * \return true if the rectangle seems ok
	 */
	template <typename T>
	inline
	bool validate_rect(cv::Rect_<T>& rect, cv::Mat& boundry) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");

		auto valid_rectangle = [rect](cv::Rect_<T>& boundry)-> bool {

			if (!(validate_rect(rect) & validate_rect(boundry)))
				return false;

			if (!validate_rect(rect))
				return false;

			return (rect & boundry).area() == boundry.area();

		};

		return valid_rectangle(cv::Rect(cv::Point(0,0), boundry.size()));

	}

	/**
	 * \brief Validates a vector of points that all containing points are above zero in both x and y
	 * \tparam T The type
	 * \param vec The vector of points to validate
	 * \return true if all pixels are above 0 in both x and y, otherwise false
	 */
	template <typename T>
	inline
	bool valid_pix_vec(std::vector<cv::Point_<T>>& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		
		if (vec.empty())
			return false;

		auto valid_pix = [](cv::Point_<T>& p) {
			return p.x < 0 || p.y < 0;
		};

		auto it = find_if(vec.begin(), vec.end(), valid_pix);

		return it != vec.end();


	}




}
