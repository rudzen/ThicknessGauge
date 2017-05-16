#pragma once
#include "LineData.h"

class LineLaserData : public LineData {
public:
	virtual ~LineLaserData();

private:

	// Left side of the laser
	cv::Vec2f left_;

	// Right side of the laser
	cv::Vec2f right_;

	// Center points of the laser
	cv::Vec2f center_;

public:

	void pointsToLine(cv::Mat& originalImage, Side side) override;

	void appendEdgePoints(cv::Vec2f& toAppend, Side side);

};


inline LineLaserData::~LineLaserData() {

}

inline void LineLaserData::pointsToLine(cv::Mat& originalImage, Side side) {

	cv::Point2f pLeft(left_[0], left_[1]);
	cv::Point2f pRight(right_[0], left_[1]);
	cv::Point2f pCenter(center_[0], center_[1]);

	cv::LineIterator itLeft(originalImage, pLeft, pCenter, 8);
	cv::LineIterator itRight(originalImage, pCenter, pRight, 8);

	leftPoints_.reserve(itLeft.count);
	rightPoints_.reserve(itRight.count);

	for (auto i = 0; i < itLeft.count; i++, ++itLeft)
		leftPoints_.push_back(itLeft.pos());

	for (auto i = 0; i < itRight.count; i++, ++itRight)
		rightPoints_.push_back(itRight.pos());

}

inline void LineLaserData::appendEdgePoints(cv::Vec2f& toAppend, Side side) {

	switch (side) {
	case Side::Left:
		left_ += toAppend;
		break;
	case Side::Right:
		right_ += toAppend;
		break;
	case Side::Center:
		center_ += toAppend;
		break;
	}
}

