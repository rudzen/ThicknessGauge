#include "LineCalc.h"
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

bool LineCalc::intersection(cv::Vec4f& border, cv::Vec4f& line, cv::Point2f& result) const {
	return intersection(cv::Point2f(border[0], border[1]), cv::Point2f(line[0], line[1]), cv::Point2f(border[2], border[3]), cv::Point2f(line[2], line[3]), result);
}


#endif
