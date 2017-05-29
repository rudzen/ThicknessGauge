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

	double innerAngle(cv::Point2d& p1, cv::Point2d& p2, cv::Point2d& c) {

		auto dist1 = cv::sqrt((p1.x - c.x) * (p1.x - c.x) + (p1.y - c.y) * (p1.y - c.y));
		auto dist2 = cv::sqrt((p2.x - c.x) * (p2.x - c.x) + (p2.y - c.y) * (p2.y - c.y));

		double Ax, Ay;
		double Bx, By;

		auto Cx = c.x;
		auto Cy = c.y;

		// checking for closest point to c
		if (dist1 < dist2) {
			Bx = p1.x;
			By = p1.y;
			Ax = p2.x;
			Ay = p2.y;
		} else {
			Bx = p2.x;
			By = p2.y;
			Ax = p1.x;
			Ay = p1.y;
		}

		auto Q1 = Cx - Ax;
		auto Q2 = Cy - Ay;
		auto P1 = Bx - Ax;
		auto P2 = By - Ay;

		auto A = acos((P1 * Q1 + P2 * Q2) / (cv::sqrt(P1 * P1 + P2 * P2) * cv::sqrt(Q1 * Q1 + Q2 * Q2)));

		A *= 180 / CV_PI;

		return A;
	}
#endif


};
