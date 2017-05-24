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

	static bool adjustMarkingRect(cv::Rect2f& markingRect, cv::Vec4f& intersectionPoints, double buffer);

	static bool adjustBaseLines(cv::Vec4f& baseLines, cv::Vec4f& intersectionPoints, double buffer);

	static double computeRealIntensityLine(cv::Mat& image, std::vector<cv::Point2d>& output, float upperLimit, float lowerLimit, std::string filename, float fileOffsetY) {

		// generate vector with all X
		if (!output.empty())
			output.clear();

		auto cols = image.cols;
		auto rows = image.rows;

		output.reserve(cols * rows);

		//cv::Mat result(image.size(), image.type());

		cv::Mat C;

		for (auto i = 0; i < image.cols; i++) {
			output.push_back(cv::Point2f(static_cast<float>(i), 0.0f));
			auto B = cv::Mat(image, cv::Rect2f(static_cast<float>(i), lowerLimit, 1.0f, upperLimit));
			B.copyTo(C);
			auto m = cv::moments(C, false);
			//int x = m.m10 / m.m00;
			auto y = m.m01 / m.m00;
			//cout << "intensity at x [x-pos / centroid]: [" << i << " / " << y << ']' << endl;
			if (y > 0) {
				output.back().y = y;
			}
		}

		//std::cout << "intensity lines: " << output << std::endl;

		//auto m = cv::moments(image, false);
		//int x = m.m10 / m.m00;
		//int y = m.m01 / m.m00;

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
