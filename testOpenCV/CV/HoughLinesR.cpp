#include <opencv2/core.hpp>
#include "HoughLinesR.h"

void HoughLinesR::computeMeta() {

	if (allLines_.empty())
		return;

	auto size = allLines_.size();

	rightLines_.clear();
	rightLines_.reserve(size);

	leftLines_.clear();
	leftLines_.reserve(size);

	auto center = image_.cols / 2;

	for (auto& a : allLines_) {
		a.slobe = (a.entry[3] - a.entry[1]) / (a.entry[2] - a.entry[0]);
		if (a.points.first.x < center)
			leftLines_.push_back(a);
		else
			rightLines_.push_back(a);
	}

	auto lSize = leftLines_.size();
	auto rSize = rightLines_.size();

	if (lSize + rSize == 0)
		throw NoLineDetectedException("No marking lines detected.");

	if (lSize == 0)
		throw NoLineDetectedException("No marking left line detected.");

	if (rSize == 0)
		throw NoLineDetectedException("No marking right line detected.");

	for (auto& left : leftLines_) {
		cv::LineIterator it(image_, left.points.first, left.points.second, 8);
		left.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++ , ++it)
			left.elements.push_back(it.pos());
	}

	if (lSize > 1)
		sort(leftLines_.begin(), leftLines_.end(), lineVsizeSort);

	for (auto& right : rightLines_) {
		cv::LineIterator it(image_, right.points.first, right.points.second, 8);
		right.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++ , ++it)
			right.elements.push_back(it.pos());
	}

	if (rSize > 1)
		sort(rightLines_.begin(), rightLines_.end(), lineVsizeSort);

}
