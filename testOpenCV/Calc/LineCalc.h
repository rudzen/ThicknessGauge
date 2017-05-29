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

	bool computeIntersectionPoints(cv::Vec4f horizontalLine, const cv::Vec4f& leftBorder, const cv::Vec4f& rightBorder, cv::Vec4f& output) const;

	static void adjustMarkingRect(cv::Rect2d& markingRect, cv::Vec4f& intersectionPoints, double buffer);

	static void adjustBaseLines(cv::Vec4f& baseLines, cv::Vec4f& intersectionPoints, double buffer);

	static double computeRealIntensityLine(cv::Mat& image, std::vector<cv::Point2d>& output, double upperLimit, double lowerLimit, std::string filename, double fileOffsetY) {

		// generate vector with all X
		if (!output.empty())
			output.clear();

		auto cols = image.cols;
		auto rows = image.rows;

		output.reserve(cols * rows);

		//cv::Mat result(image.size(), image.type());

		cv::Mat C;

		for (auto i = 0; i < image.cols; i++) {
			output.push_back(cv::Point2d(static_cast<double>(i), 0.0));
			auto B = cv::Mat(image, cv::Rect2f(static_cast<double>(i), lowerLimit, 1.0, upperLimit));
			B.copyTo(C);
			auto m = cv::moments(C, false);
			//int x = m.m10 / m.m00;
			auto y = m.m01 / m.m00;
			//cout << "intensity at x [x-pos / centroid]: [" << i << " / " << y << ']' << endl;
			if (y > 0)
				output.back().y = y;
		}

		std::ofstream file(filename + ".intensitet.txt");
		std::ofstream fileY(filename + ".intensitet_y.txt");
		std::ofstream fileButt(filename + ".intensitet_y_korigeret.txt");
		double avg[2] = {0.0, 0.0};
		for (auto& h : output) {
			//out = to_string(h.y);
			avg[0] += h.y;
			avg[1] += rows - h.y;
			file << h.x << ' ' << (h.y + fileOffsetY) << '\n';
			fileY << h.y + fileOffsetY << '\n';
			fileButt << rows - (h.y + fileOffsetY) << '\n';
		}

		avg[0] /= output.size();
		avg[1] /= output.size();

		fileY << "avg:" << avg[0] << '\n';
		fileButt << "avg:" << avg[1] << '\n';

		file.close();
		fileY.close();
		fileButt.close();

		return avg[1];

	}

#endif


};
