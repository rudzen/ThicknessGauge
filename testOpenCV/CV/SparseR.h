#pragma once
#include <opencv2/stitching/detail/warpers.hpp>
#include "BaseR.h"
#include "Util/Util.h"

template <class T>
class SparseR : public BaseR<float> {

	std::vector<cv::Point_<T>> allSparse_;
	std::vector<cv::Point_<T>> allTotal_;

	struct elementXsort {
		bool operator()(cv::Point_<T>& pt1, cv::Point_<T>& pt2) const { return pt1.x < pt2.x; }
	} sortX;

public:

	const std::vector<cv::Point_<T>>& allSparse() const {
		return allSparse_;
	}
	
	void allSparse(const std::vector<cv::Point_<T>>& allSparse) {
		allSparse_ = allSparse;
	}

	const std::vector<cv::Point_<T>>& allTotal() const {
		return allTotal_;
	}

	void allTotal(const std::vector<cv::Point_<T>>& allTotal) {
		allTotal_ = allTotal;
	}

	void initialize();

	bool generateSparse();

	size_t addToTotal(const std::vector<cv::Point_<T>>& toAdd);

};

template <class T>
size_t SparseR<T>::addToTotal(const std::vector<cv::Point_<T>>& toAdd) {
	allTotal_.reserve(toAdd.size() + allTotal_.size());
	allTotal_.insert(allTotal_.begin(), toAdd.begin(), toAdd.end());
	return allTotal_.size();
}

template <class T>
void SparseR<T>::initialize() {
	allTotal_.clear();
	allSparse_.clear();
}

template <class T>
bool SparseR<T>::generateSparse() {

	if (!allSparse_.empty())
		allSparse_.clear();

	allSparse_.reserve(image_.cols);

	if (allTotal_.empty())
		return false;

	// sort the list in X
	std::sort(allTotal_.begin(), allTotal_.end(), sortX);

	auto y = 0;
	auto count = 0;
	auto highest = 0;

	auto x = allTotal_.front().x;

	for (auto& p : allTotal_) {
		if (p.x != x) {
			if (count > 0) {
				allSparse_.push_back(cv::Point(x, image_.rows - y));
				count = 0;
			}
			highest = 0;
		}
		auto intensity = image_.at<unsigned char>(p);
		if (intensity >= highest) {
			highest = p.y;
			x = p.x;
			y = p.y;
		}
		count++;
	}

	allTotal_.clear();

	return allSparse_.empty() ^ true;

}
