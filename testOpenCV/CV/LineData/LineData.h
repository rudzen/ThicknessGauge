#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "../../_cv.h"
#include <ostream>
#include "LineData.h"

using namespace _cv;

// super class for both line classes..
class LineData {
	
protected:
	~LineData() = default;

	enum class Side {
		Left,
		Right,
		Center // only used for laser line data, base lines dont have a center
	};

	/**
	* \brief Convert both left and right vector points to vectors of regular x/y points
	* \param originalImage PH image, not used for anything
	* \param side The side to compute for
	*/
	virtual void pointsToLine(cv::Mat& originalImage, Side side) = 0;

	void calcYAvg();

	template <Side side>
	void appendPoint(cv::Point2f& point);

	// The left side points which makes up the left base line
	std::vector<cv::Point2f> leftPoints_;

	// The right side points which makes up the right base line
	std::vector<cv::Point2f> rightPoints_;

	// The left side Y avg value
	float leftAvgY_;

	// The right side Y avg value
	float rightAvgY_;

	LineData() {
		leftAvgY_ = 0.0f;
		rightAvgY_ = 0.0f;
	}

	LineData(const LineData& other)
		: leftPoints_(other.leftPoints_),
		  rightPoints_(other.rightPoints_),
		  leftAvgY_(other.leftAvgY_),
		  rightAvgY_(other.rightAvgY_) {
	}

	LineData& operator=(const LineData& other) {
		if (this == &other)
			return *this;
		leftPoints_ = other.leftPoints_;
		rightPoints_ = other.rightPoints_;
		leftAvgY_ = other.leftAvgY_;
		rightAvgY_ = other.rightAvgY_;
		return *this;
	}

public:
	const std::vector<cv::Point2f>& leftPoints() const {
		return leftPoints_;
	}

	void leftPoints(const std::vector<cv::Point2f>& leftPoints) {
		leftPoints_ = leftPoints;
	}

	const std::vector<cv::Point2f>& rightPoints() const {
		return rightPoints_;
	}

	void rightPoints(const std::vector<cv::Point2f>& rightPoints) {
		rightPoints_ = rightPoints;
	}

	const float& leftAvgY() const {
		return leftAvgY_;
	}

	void leftAvgY(float leftAvgY) {
		leftAvgY_ = leftAvgY;
	}

	const float& rightAvgY() const {
		return rightAvgY_;
	}

	void rightAvgY(float rightAvgY) {
		rightAvgY_ = rightAvgY;
	}
};

template <LineData::Side side>
void LineData::appendPoint(cv::Point2f& point) {
	switch (side) {
	case Side::Left:
		leftPoints_.push_back(point);
		break;
	case Side::Right:
		rightPoints_.push_back(point);
		break;
	case Side::Center:
	default:
		std::cerr << "Error while adding point, center is not a valid side.";
	}
}

/**
* \brief Computes the average Y values of both lines
*/
inline void LineData::calcYAvg() {

	auto sum = 0.0f;

	for (auto& p : leftPoints_)
		sum += p.y;

	leftAvgY_ = sum / static_cast<float>(leftPoints_.size());

	sum = 0.0f;

	for (auto& p : rightPoints_)
		sum += p.y;

	rightAvgY_ = sum / static_cast<float>(rightPoints_.size());

}
