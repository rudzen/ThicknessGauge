﻿#include "LineCalc.h"
#include <opencv2/core.hpp>

#ifdef CV_VERSION

bool LineCalc::intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f& r) const {

	auto d1 = p1 - o1;
	auto d2 = p2 - o2;

	auto cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8) return false;

	auto x = o2 - o1;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

bool LineCalc::intersection(const cv::Vec4f& border, cv::Vec4f& line, cv::Point2f& result) const {
	return intersection(cv::Point2f(border[0], border[1]), cv::Point2f(line[0], line[1]), cv::Point2f(border[2], border[3]), cv::Point2f(line[2], line[3]), result);
}

bool LineCalc::computeIntersectionPoints(cv::Vec4f horizontalLine, const cv::Vec4f& leftBorder, const cv::Vec4f& rightBorder, cv::Vec4f& output) const {

	cv::Point2f leftIntersection;
	cv::Point2f rightIntersection;

	horizontalLine[0] = leftBorder[0];
	horizontalLine[2] = rightBorder[2];

	auto intersectsLeft = intersection(leftBorder, horizontalLine, leftIntersection);
	auto intersectsRight = intersection(rightBorder, horizontalLine, rightIntersection);

	if (!intersectsLeft & intersectsRight) {
		Util::loge("Error in intersection detection.");
		return false;
	}

	output[0] = leftIntersection.x;
	output[1] = leftIntersection.y;
	output[2] = rightIntersection.x;
	output[3] = rightIntersection.y;

	return true;

}

#endif

bool LineCalc::adjustMarkingRect(cv::Rect2f& markingRect, cv::Vec4f intersectionPoints, uint buffer) {

	if (markingRect.x == 0)
		return false;

	if (markingRect.width == 0)
		return false;

	markingRect.x = intersectionPoints[0] + buffer;
	markingRect.width = intersectionPoints[2] - markingRect.x - buffer;

	return true;
}

bool LineCalc::adjustBaseLines(cv::Vec4f& baseLines, cv::Vec4f& intersectionPoints, uint buffer) {
	
	baseLines[0] = intersectionPoints[0] - buffer;
	baseLines[2] = intersectionPoints[2] + buffer;

	return true;

}