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

	bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f& r) const;

	bool intersection(const cv::Vec4f& border, cv::Vec4f& line, cv::Point2f& result) const;

	bool computeIntersectionPoints(cv::Vec4f horizontalLine, const cv::Vec4f& leftBorder, const cv::Vec4f& rightBorder, cv::Vec4d& output) const;

	static void adjustMarkingRect(cv::Rect2d& markingRect, cv::Vec4d& intersectionPoints, double buffer);

	static void adjustBaseLines(cv::Vec4d& baseLines, cv::Vec4d& intersectionPoints, double buffer);

	static double computeRealIntensityLine(cv::Mat& image, std::vector<cv::Point2d>& output, double upperLimit, double lowerLimit, std::string filename, double fileOffsetY);

	double angleBetweenLines(cv::Point2d& p1, cv::Point2d& p2) const {
		return atan((p1.y - p2.y) / (p2.x - p1.x) * 180 / CV_PI);
	}

	double innerAngle(cv::Point2d& p1, cv::Point2d& p2, cv::Point2d& c) const;
#endif


};
