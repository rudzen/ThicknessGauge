#pragma once
#include <opencv2/core.hpp>
#include "../namespaces/tg.h"
#include <opencv2/shape/shape_transformer.hpp>
#include "CV/LaserR.h"

using namespace tg;

class LineCalc {

public:

#ifdef CV_VERSION

	bool intersection(cv::Point2d o1, cv::Point2d p1, cv::Point2d o2, cv::Point2d p2, cv::Point2d& r) const;

	bool intersection(const cv::Vec4d& border, cv::Vec4d& line, cv::Point2d& result) const;

	bool computeIntersectionPoints(cv::Vec4d horizontal_line, const cv::Vec4d& left_border, const cv::Vec4d& right_border, cv::Vec4d& output) const;

	static void adjustMarkingRect(cv::Rect2d& marking_rect, cv::Vec4d& intersection_points, double buffer);

	static void adjustBaseLines(cv::Vec4d& base_lines, cv::Vec4d& intersection_points, double buffer);

	static double computeRealIntensityLine(cv::Mat& image, std::vector<cv::Point2d>& output, int upper_limit, int lower_limit);

	double angleBetweenLines(cv::Point2d& p1, cv::Point2d& p2) const {
		return atan((p1.y - p2.y) / (p2.x - p1.x) * 180 / CV_PI);
	}

	double innerAngle(cv::Point2d& p1, cv::Point2d& p2, cv::Point2d& c) const;
#endif


};
