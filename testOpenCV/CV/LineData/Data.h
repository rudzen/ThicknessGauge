#pragma once
#include <array>
#include "LineData.h"

class Data {

	cv::Size totalSize_;

	std::vector<LineData> lines_;

	std::vector<cv::Rect2d> rectangles_;

	uint exposure_;

public:

	Data(): exposure_(0) { }

	explicit Data(const cv::Size size) : totalSize_(size), exposure_(0) {
		lines_[Left].setSide(Left);
		lines_[Right].setSide(Right);
		lines_[Center].setSide(Center);
	}

	const LineData& getLine(Side side) const {
		return lines_[side];
	}

	std::vector<cv::Rect2d>& getRectangles() {
		return rectangles_;
	}

	const cv::Rect2d& getRectangle(Side side) const {
		return rectangles_[side];
	}

	void setRectangle(Side side, cv::Rect2d rectangle) {
		rectangles_[side] = rectangle;
	}

	cv::Size getTotalSize() const {
		return cv::Size(totalSize_);
	}

	void setTotalSize(cv::Size totalSize) {
		totalSize_ = totalSize;
	}

	uint getExposure() const {
		return exposure_;
	}

	void setExposure(uint exposure) {
		this->exposure_ = exposure;
	}
};
