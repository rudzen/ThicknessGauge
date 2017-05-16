#pragma once
#include "LineData.h"

using namespace _cv;

/**
 * \brief Container class for line data
 */
class LineBaseData : public LineData {

	// The left side outer points which makes up the left side of the base line
	cv::Vec4f left_;

	// The right side outer points which makes up the right side of the base line
	cv::Vec4f right_;

	void pointsToLine(cv::Mat& originalImage, Side side) override;

public:
	virtual ~LineBaseData() = default;

	LineBaseData() : LineData() {
	}

	LineBaseData(const LineBaseData& other)
		: LineData(other),
		  left_(other.left_),
		  right_(other.right_) {
	}

	// ReSharper disable CppPossiblyUninitializedMember
	LineBaseData(cv::Vec4f left, cv::Vec4f right)
	// ReSharper restore CppPossiblyUninitializedMember
		: LineData(), left_(left),
		  right_(right) {
	}

	LineBaseData& operator=(const LineBaseData& other) {
		if (this == &other)
			return *this;
		leftPoints_ = other.leftPoints_;
		rightPoints_ = other.rightPoints_;
		left_ = other.left_;
		right_ = other.right_;
		return *this;
	}

	friend bool operator==(const LineBaseData& lhs, const LineBaseData& rhs) {
		return lhs.left_ == rhs.left_
			&& lhs.right_ == rhs.right_;
	}

	friend LineBaseData operator+(const LineBaseData& lhs, const LineBaseData& rhs) {
		// lel, no + operator for vec4f
		return LineBaseData(cv::Vec4f(lhs.left_[0] + rhs.left_[0],
		                              lhs.left_[1] + rhs.left_[1],
		                              lhs.left_[2] + rhs.left_[2],
		                              lhs.left_[3] + rhs.left_[3]),
		                    cv::Vec4f(lhs.right_[0] + rhs.right_[0],
		                              lhs.right_[1] + rhs.right_[1],
		                              lhs.right_[2] + rhs.right_[2],
		                              lhs.right_[3] + rhs.right_[3]));
	}

	friend bool operator!=(const LineBaseData& lhs, const LineBaseData& rhs) {
		return !(lhs == rhs);
	}

	friend std::ostream& operator<<(std::ostream& os, const LineBaseData& obj) {
		return os
			<< "\nleft_ [pts]: " << obj.left_ << '[' << obj.leftPoints_ << ']'
			<< "\nright_[pts]: " << obj.right_ << '[' << obj.rightPoints_ << ']';
	}

	void generatePointLines(cv::Mat& originalImage);

	void appendEdgePoints(cv::Vec4f& toAppend, Side side);

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

};

/**
 * \brief Convert both left and right vector points to vectors of regular x/y points
 * \param originalImage PH image, not used for anything
 * \param side The side to compute for
 */
inline void LineBaseData::pointsToLine(cv::Mat& originalImage, Side side) {

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
inline void LineBaseData::generatePointLines(cv::Mat& originalImage) {

	pointsToLine(originalImage, Side::Left);

	pointsToLine(originalImage, Side::Right);

	calcYAvg();

}

inline void LineBaseData::appendEdgePoints(cv::Vec4f& toAppend, Side side) {

	switch (side) {
	case Side::Left:
		left_ += toAppend;
		break;
	case Side::Right:
		right_ += toAppend;
		break;
	case Side::Center:
	default:
		std::cerr << "Error while appending points, side does not exist for this element." << std::endl;
	}
}

