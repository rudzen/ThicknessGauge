#pragma once
#include <opencv2/core.hpp>
#include "../tg.h"
#include <opencv2/shape/shape_transformer.hpp>
#include "CV/LaserR.h"

using namespace tg;

class LineCalc {

public:

#ifdef CV_VERSION

	LineCalc() {
	}

	bool intersection(cv::Point2d o1, cv::Point2d p1, cv::Point2d o2, cv::Point2d p2, cv::Point2d& r) const;

	bool intersection(const cv::Vec4d& border, cv::Vec4d& line, cv::Point2d& result) const;

	bool computeIntersectionPoints(cv::Vec4d horizontalLine, const cv::Vec4d& leftBorder, const cv::Vec4d& rightBorder, cv::Vec4d& output) const;

	static void adjustMarkingRect(cv::Rect2d& markingRect, cv::Vec4d& intersectionPoints, double buffer);

	static void adjustBaseLines(cv::Vec4d& baseLines, cv::Vec4d& intersectionPoints, double buffer);

	static double computeRealIntensityLine(cv::Mat& image, std::vector<cv::Point2d>& output, double upperLimit, double lowerLimit, std::string filename, double fileOffsetY);

	double angleBetweenLines(cv::Point2d& p1, cv::Point2d& p2) const {
		return atan((p1.y - p2.y) / (p2.x - p1.x) * 180 / CV_PI);
	}

	double innerAngle(cv::Point2d& p1, cv::Point2d& p2, cv::Point2d& c) const;
#endif


};
