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
		auto contourComparator = [](std::vector<cv::Point_<T>> a, std::vector<cv::Point_<T>> b) {
			return contourArea(a) > contourArea(b);
		};
		std::sort(contours.begin(), contours.end(), contourComparator);
	}

	template <typename T>
	inline
	void sort_pixels_x_ascending(std::vector<cv::Point_<T>>& vector) {
		auto x_comparator = [](cv::Point_<T> p1, cv::Point_<T> p2) {
			return p1.x < p2.x;
		};
		std::sort(vector.begin(), vector.end(), x_comparator);
	}

	template <typename T>
	inline
	void sort_pixels_x_descending(std::vector<cv::Point_<T>>& vector) {
		auto x_comparator = [](cv::Point_<T> p1, cv::Point_<T> p2) {
			return p1.x > p2.x;
		};
		std::sort(vector.begin(), vector.end(), x_comparator);
	}

	template <typename T>
	inline
	void sort_pixels_y_ascending(std::vector<cv::Point_<T>>& vector) {
		auto x_comparator = [](cv::Point_<T> p1, cv::Point_<T> p2) {
			return p1.y < p2.y;
		};
		std::sort(vector.begin(), vector.end(), x_comparator);
	}

	template <typename T>
	inline
	void sort_pixels_y_descending(std::vector<cv::Point_<T>>& vector) {
		auto x_comparator = [](cv::Point_<T> p1, cv::Point_<T> p2) {
			return p1.y > p2.y;
		};
		std::sort(vector.begin(), vector.end(), x_comparator);
	}

}
