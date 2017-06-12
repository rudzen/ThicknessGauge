#pragma once

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

}
