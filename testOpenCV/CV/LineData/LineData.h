#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#ifndef CV_VERSION
#include <array>
#endif
#include "../../tg.h"
#include "LineData.h"

using namespace tg;

class LineData {

public:
	explicit LineData(Side side) : avg_(0.0)
	                            , side_(side) {}

	void computeAvg();

	void setOffset(XY xy, double offset);

	double getOffset(XY xy);

	const std::vector<cv::Point2d>& points() const {
		return points_;
	}

	void set_points(const std::vector<cv::Point2d>& points) {
		points_ = points;
	}

	double getAvg() const {
		return avg_;
	}

	void setAvg(double avg) {
		avg_ = avg;
	}

	Side getSide() const {
		return side_;
	}

	void setSide(Side side) {
		side_ = side;
	}

protected:

	std::vector<cv::Point2d> points_;

	// offset in x and y, where 0 = X and 1 = Y
#ifdef CV_VERSION
	cv::Vec2d offset_;
#else
	std::array<double, 2> offset_;
#endif

	double avg_;

	Side side_;

};

inline void LineData::setOffset(XY xy, double offset) {
	offset_[xy] = offset;
}

inline double LineData::getOffset(XY xy) {
	return offset_[xy];
}

/**
* \brief Computes the average Y values of both lines
*/
inline void LineData::computeAvg() {

	auto sum = 0.0;

	for (auto& p : points_)
		sum += p.y;

	avg_ = sum / static_cast<float>(points_.size());

}
