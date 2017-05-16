#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "../_cv.h"
#include <ostream>

using namespace _cv;

/**
 * \brief Container class for line data
 */
class LineData {

	enum class Side {
		Left,
		Right
	};

	// The left side points which makes up the border of the marking
	vector<cv::Point2f> leftPoints_;

	// The right side points which makes up the border of the marking
	vector<cv::Point2f> rightPoints_;

	// The left side outer points which makes up the border of the marking
	cv::Vec4f left_;

	// The right side outer points which makes up the border of the marking
	cv::Vec4f right_;

	// The left side Y avg value
	float leftAvgY_;

	float rightAvgY_;

	void pointsToLine(cv::Mat& originalImage, Side side);

	void calcYAvg();

public:

	LineData() {
		leftAvgY_ = 0.0f;
		rightAvgY_ = 0.0f;
	}

	LineData(const LineData& other)
		: leftPoints_(other.leftPoints_),
		  rightPoints_(other.rightPoints_),
		  left_(other.left_),
		  right_(other.right_),
		  leftAvgY_(other.leftAvgY_),
		  rightAvgY_(other.rightAvgY_) {
	}

	// ReSharper disable CppPossiblyUninitializedMember
	LineData(cv::Vec4f left, cv::Vec4f right)
	// ReSharper restore CppPossiblyUninitializedMember
		: left_(left),
		  right_(right) {
		LineData();
	}

	LineData& operator=(const LineData& other) {
		if (this == &other)
			return *this;
		leftPoints_ = other.leftPoints_;
		rightPoints_ = other.rightPoints_;
		left_ = other.left_;
		right_ = other.right_;
		return *this;
	}

	friend bool operator==(const LineData& lhs, const LineData& rhs) {
		return lhs.left_ == rhs.left_
			&& lhs.right_ == rhs.right_;
	}

	friend LineData operator+(const LineData& lhs, const LineData& rhs) {
		// lel, no + operator for vec4f
		return LineData(cv::Vec4f(lhs.left_[0] + rhs.left_[0],
		                          lhs.left_[1] + rhs.left_[1],
		                          lhs.left_[2] + rhs.left_[2],
		                          lhs.left_[3] + rhs.left_[3]),
		                cv::Vec4f(lhs.right_[0] + rhs.right_[0],
		                          lhs.right_[1] + rhs.right_[1],
		                          lhs.right_[2] + rhs.right_[2],
		                          lhs.right_[3] + rhs.right_[3]));
	}

	friend bool operator!=(const LineData& lhs, const LineData& rhs) {
		return !(lhs == rhs);
	}

	friend ostream& operator<<(ostream& os, const LineData& obj) {
		return os
			<< "\nleft_ [pts]: " << obj.left_ << '[' << obj.leftPoints_ << ']'
			<< "\nright_[pts]: " << obj.right_ << '[' << obj.rightPoints_ << ']';
	}

	void generatePointLines(cv::Mat& originalImage);

	const cv::Vec4f& left() const {
		return left_;
	}

	void left(const cv::Vec4f& left) {
		left_ = left;
	}

	const cv::Vec4f& right() const {
		return right_;
	}

	void right(const cv::Vec4f& right) {
		right_ = right;
	}

	const vector<cv::Point2f>& leftPoints() const {
		return leftPoints_;
	}

	void leftPoints(const vector<cv::Point2f>& leftPoints) {
		leftPoints_ = leftPoints;
	}

	const vector<cv::Point2f>& rightPoints() const {
		return rightPoints_;
	}

	void rightPoints(const vector<cv::Point2f>& rightPoints) {
		rightPoints_ = rightPoints;
	}

};

/**
 * \brief Convert both left and right vector points to vectors of regular x/y points
 * \param originalImage PH image, not used for anything
 * \param side The side to compute for
 */
inline void LineData::pointsToLine(cv::Mat& originalImage, Side side) {

	auto& v = side == Side::Left ? left_ : right_;

	cv::Point2f p1(v[0], v[1]);
	cv::Point2f p2(v[2], v[3]);

	cv::LineIterator it(originalImage, p1, p2, 8);

	auto& l = side == Side::Left ? leftPoints_ : rightPoints_;

	l.reserve(it.count);

	for (auto i = 0; i < it.count; i++ , ++it)
		l.push_back(it.pos());
}

/**
 * \brief Generates the point lines for contained points.
 * \param originalImage PH image, not really used
 */
inline void LineData::generatePointLines(cv::Mat& originalImage) {

	pointsToLine(originalImage, Side::Left);

	pointsToLine(originalImage, Side::Right);

	calcYAvg();

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
