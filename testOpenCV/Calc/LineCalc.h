#pragma once
#include <opencv2/core.hpp>
#include "../namespaces/tg.h"
#include <opencv2/shape/shape_transformer.hpp>
#include "CV/LaserR.h"

using namespace tg;

class LineCalc {

public:

#ifdef CV_VERSION

	bool computeIntersectionPoints(cv::Vec4d horizontal_line, const cv::Vec4d& left_border, const cv::Vec4d& right_border, cv::Vec4d& output) const;

	static void adjustMarkingRect(cv::Rect2d& marking_rect, cv::Vec4d& intersection_points, double buffer);

	static void adjustBaseLines(cv::Vec4d& base_lines, cv::Vec4d& intersection_points, double buffer);

	static double computeRealIntensityLine(cv::Mat& image, std::vector<cv::Point2d>& output, int upper_limit, int lower_limit);
	
#endif


};
