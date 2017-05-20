#pragma once
#include <opencv2/core.hpp>

class LineCalc {
public:

#ifdef CV_VERSION
	
	bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f& r) const;

	bool intersection(cv::Vec4f& border, cv::Vec4f& line, cv::Point2f& result) const;

#endif



};
