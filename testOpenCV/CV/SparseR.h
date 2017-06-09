#pragma once
#include <opencv2/stitching/detail/warpers.hpp>
#include "BaseR.h"

class SparseR : public BaseR {

	std::vector<cv::Point2d> allSparse_;
	std::vector<cv::Point2d> allTotal_;

	struct elementXsort {
		bool operator()(cv::Point2d& pt1, cv::Point2d& pt2) const { return pt1.x < pt2.x; }
	} sortX;

public:

	const std::vector<cv::Point2d>& allSparse() const {
		return allSparse_;
	}
	
	void allSparse(const std::vector<cv::Point2d>& allSparse) {
		allSparse_ = allSparse;
	}

	const std::vector<cv::Point2d>& allTotal() const {
		return allTotal_;
	}

	void allTotal(const std::vector<cv::Point2d>& allTotal) {
		allTotal_ = allTotal;
	}

	void initialize();

	bool generateSparse();

	size_t addToTotal(const std::vector<cv::Point2d>& toAdd);

};

inline size_t SparseR::addToTotal(const std::vector<cv::Point2d>& toAdd) {
	allTotal_.reserve(toAdd.size() + allTotal_.size());
	allTotal_.insert(allTotal_.begin(), toAdd.begin(), toAdd.end());
	return allTotal_.size();
}

inline void SparseR::initialize() {
	allTotal_.clear();
	allSparse_.clear();
}

inline bool SparseR::generateSparse() {

	if (!allSparse_.empty())
		allSparse_.clear();

	allSparse_.reserve(image_.cols);

	if (allTotal_.empty())
		return false;

	// sort the list in X
	std::sort(allTotal_.begin(), allTotal_.end(), sortX);

	auto y = 0.0;
	auto count = 0;
	auto highest = 0.0;

	auto x = allTotal_.front().x;

	for (auto& p : allTotal_) {
		if (p.x != x) {
			if (count > 0) {
				allSparse_.emplace_back(cv::Point2d(x, image_.rows - y));
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
