#pragma once
#include <array>
#include "LineData.h"

class Data {
public:
	explicit Data(const cv::Size size) : exposure(0) {
		lines_[Left].setSide(Left);
		lines_[Right].setSide(Right);
		lines_[Center].setSide(Center);
	}

	std::array<LineData, 3> lines_;

	uint exposure;

};
