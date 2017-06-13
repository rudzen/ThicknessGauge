#pragma once

#include <opencv2/core/types.hpp>

#if defined(_MSC_VER) && !defined(inline)
#define inline __forceinline
#elif defined(__GNUC__) && !defined(inline)
#define inline __always_inline
#endif

namespace stl {

	template <typename T>
	void copyVector(T& source, T& destination) {
		destination.reserve(source.size() + destination.size());
		destination.insert(destination.begin(), source.begin(), source.end());
	}

	template <typename T>
	void copyVector(const T& source, T& destination) {
		destination.reserve(source.size() + destination.size());
		destination.insert(destination.begin(), source.begin(), source.end());
	}

	template <typename T>
	inline
	void sort_contours(std::vector<std::vector<cv::Point_<T>>>& contours) {
		auto contourComparator = [](std::vector<cv::Point_<T>> a, std::vector<cv::Point_<T>> b) {
			return contourArea(a) > contourArea(b);
		};
		std::sort(contours.begin(), contours.end(), contourComparator);
	}

#ifdef CV_VERSION

	/**
	 * \brief Populates the vector with index based X coordinates and zero value in Y
	 * \tparam T The type
	 * \param vec The vector to fill
	 * \param limit The limit for X
	 */
	template <typename T>
	inline
	void populate_x(std::vector<cv::Point_<T> >& vec, const size_t limit) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		vec.clear();
		vec.reserve(limit);
		for (auto i = 0; i < limit; i++)
			vec.emplace_back(cv::Point_<T>(i, 0));

		vec.shrink_to_fit();
	}

	/**
	 * \brief Reset all Y values in vector of points to zero
	 * \tparam T The type
	 * \param vec The vector to reset all Y values in
	 */
	template <typename T>
	inline
	void reset_point_y(std::vector<cv::Point_<T> >& vec) {
		static_assert(std::is_fundamental<T>::value, "type is only possible for fundamental types.");
		T zero = static_cast<T>(0);
		for (auto& v : vec) {
			v.y = zero;
		}
		//for (auto i = 0; i < vec.size(); i++) {
		//	vec[i].y = zero;
		//}
			
	}


#endif


}
