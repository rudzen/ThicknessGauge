#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "BaseR.h"
#include "../namespaces/tg.h"
#include "../namespaces/calc.h"

#include "Exceptions/NoLineDetectedException.h"

/*
   |  __
   | /__\
   | X~~|			"The eternal code god
   |-\|//-.		 watches over this mess."
/|`.|'.' \			- R.A.Kohn, 2017
|,|.\~~ /||
|:||   ';||
||||   | ||
\ \|     |`.
|\X|     | |
| .'     |||
| |   .  |||
|||   |  `.| JS
||||  |   ||
||||  |   ||
`+.__._._+*/


using namespace std;

class HoughLinesR : public BaseR {

public:

	typedef struct LineV {
		cv::Vec2f entry;
		tg::linePair points;
		std::vector<cv::Point_<float>> elements;

		float slobe;

		LineV() {
			slobe = 0.0f;
		}

		LineV(cv::Vec2f entry, tg::linePair points)
			: entry(entry),
			  points(points) {
			elements.reserve(cvRound(calc::dis_manhattan(points.first.x, points.second.x, points.first.y, points.second.y)));
			slobe = 0.0f;
		}

		friend bool operator==(const LineV& lhs, const LineV& rhs) {
			return lhs.slobe == rhs.slobe
				&& lhs.entry == rhs.entry
				&& lhs.points == rhs.points
				&& lhs.elements == rhs.elements;
		}

		friend bool operator!=(const LineV& lhs, const LineV& rhs) {
			return !(lhs == rhs);
		}

		friend std::ostream& operator<<(std::ostream& os, const LineV& obj) {
			return os
				<< "entry: " << obj.entry
				<< "slobe: " << obj.slobe
				<< " points(1/2): " << obj.points.first << '/' << obj.points.second
				<< " elements: " << obj.elements;
		}
	} LineV;

	struct lineVsizeSort {
		bool operator()(LineV l1, LineV l2) const { return l1.elements.size() < l2.elements.size(); }
	} lineVsizeSort;

	struct lineVYSort {
		bool operator()(cv::Point2f p1, cv::Point2f p2) const { return p1.y < p2.y; }
	} lineVYSort;

private:

	// the output for visual display ONLY!
	cv::Mat output_;

	// the line output for the houghlines algorithm
	vector<cv::Vec2f> lines_;

	// the lines with all information
	vector<LineV> allLines_;

	// the lines located on the right side of the image
	vector<LineV> rightLines_;

	// the lines location on the left side of the image
	vector<LineV> leftLines_;

	// the x coordinates of the left "border" of the marking
	cv::Vec4d leftBorder_;

	// the x coordinates of the right "border" of the marking
	cv::Vec4d rightBorder_;

	double leftY_ = 0.0;

	/* only used for UI display through open cv*/

	int rho_;

	int theta_;

	double angle_;

	int threshold;

	const int APERTURE_MIN = 3;
	const int APERTURE_MAX = 7;

	double srn_;

	double stn_;

	double minTheta_;

	double maxTheta_;

	int iAngleLimit_;

	double angleLimit_;

public:

	HoughLinesR(const int rho, const int theta, const int threshold, const bool showWindow)
		: rho_(rho),
		  theta_(theta),
		  threshold(threshold) {
		angle_ = CV_PI / 180 * theta;
		srn_ = 0;
		stn_ = 0;
		minTheta_ = 0.0;
		maxTheta_ = CV_PI;
		angleLimit_ = 0;
		windowName = "HoughLines";
		showWindows_ = showWindow;
		if (showWindow)
			createWindow();
	}

	void computeBorders();

private:
	void createWindow() {
		namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("rho", windowName, &rho_, 3, rhocb, this);
		cv::createTrackbar("theta", windowName, &theta_, 180, thetacb, this);
		cv::createTrackbar("threshold", windowName, &threshold, 100, thresholdcb, this);
	}

	tg::linePair computePointPair(cv::Vec2f& line) const;

	tg::linePair computePointPair2(cv::Vec2f& line) const;

	void drawLines(vector<LineV>& linePairs, cv::Scalar colour);

	void drawLine(cv::Vec4d& line);

	void showOutput() const;

	void computeMeta();

	static void computeRectFromLines(vector<LineV>& input, cv::Rect2d& output);

	// callbacks

	static void rhocb(int value, void* userData);

	static void thetacb(int value, void* userData);

	static void thresholdcb(int value, void* userData);

	// getters & setters

	void setRho(int rho) {
		this->rho_ = rho;
	}

	void setTheta(int theta) {
		if (theta == 0)
			theta++;
		this->theta_ = theta;
		angle_ = degree * theta;
	}

	void setThreshold(int threshold) {
		this->threshold = threshold;
	}

public:

	int doVerticalHough();

	void setAngleLimit(double angleLimit) {
		this->angleLimit_ = angleLimit;
	}

	void setOriginal(cv::Mat& original) {
		original_ = original;
		if (showWindows_)
			cvtColor(original_, output_, CV_GRAY2BGR);
	}

	const vector<cv::Vec2f>& getLines() const {
		return lines_;
	}

	const vector<LineV>& getAllLines() const {
		return allLines_;
	}

	void setAllLines(const vector<LineV>& allLines) {
		this->allLines_ = allLines;
	}

	const vector<LineV>& getRightLines() const {
		return rightLines_;
	}

	void setRightLines(const vector<LineV>& rightLines) {
		this->rightLines_ = rightLines;
	}

	const vector<LineV>& getLeftLines() const {
		return leftLines_;
	}

	void setLeftLines(const vector<LineV>& leftLines) {
		this->leftLines_ = leftLines;
	}

	const cv::Vec4d& getLeftBorder() const {
		return leftBorder_;
	}

	const cv::Vec4d& getRightBorder() const {
		return rightBorder_;
	}

	void leftBorder(cv::Vec4d leftBorder) {
		leftBorder_ = leftBorder;
	}

	void rightBorder(cv::Vec4d rightBorder) {
		rightBorder_ = rightBorder;
	}
};

inline void HoughLinesR::computeRectFromLines(vector<LineV>& input, cv::Rect2d& output) {

	for (auto& line : input) {
		cv::Rect2f t = cv::boundingRect(line.elements);
		output.x += t.x;
		output.y += t.y;
		output.width += t.width;
	}

	const auto size = input.size();

	output.x /= size;
	output.y /= size;
	output.width /= size;

}

inline void HoughLinesR::computeBorders() {

	cv::Rect2d leftRoi(0.0, 0.0, 0.0, static_cast<double>(image_.rows));
	cv::Rect2d rightRoi(0.0, 0.0, 0.0, static_cast<double>(image_.rows));

	computeRectFromLines(leftLines_, leftRoi);
	computeRectFromLines(rightLines_, rightRoi);

	auto imgHeight = static_cast<double>(image_.rows);

	markingRect.x = leftRoi.x;
	markingRect.y = leftRoi.y;
	markingRect.width = rightRoi.x - leftRoi.x + rightRoi.width;
	markingRect.height = imgHeight;

	leftBorder_[0] = leftRoi.x;
	leftBorder_[1] = imgHeight;
	leftBorder_[2] = leftRoi.x + leftRoi.width;
	leftBorder_[3] = 0.0f;

	rightBorder_[0] = rightRoi.x;
	rightBorder_[1] = 0.0f;
	rightBorder_[2] = rightRoi.x + rightRoi.width;
	rightBorder_[3] = imgHeight;

	if (showWindows_) {
		drawLine(leftBorder_);
		drawLine(rightBorder_);
		showOutput();
	}

}

inline void HoughLinesR::rhocb(int value, void* user_data) {
	auto that = static_cast<HoughLinesR*>(user_data);
	that->setTheta(value);
	cout << cv::format("%s rho : %i\n", that->windowName, value);
}

inline void HoughLinesR::thetacb(int value, void* user_data) {
	auto that = static_cast<HoughLinesR*>(user_data);
	that->setTheta(value);
	cout << cv::format("%s theta : %i\n", that->windowName, value);
}

inline void HoughLinesR::thresholdcb(int value, void* user_data) {
	auto that = static_cast<HoughLinesR*>(user_data);
	that->setThreshold(value);
	cout << cv::format("%s threshold : %i\n", that->windowName, value);
}


inline int HoughLinesR::doVerticalHough() {

	if (!lines_.empty())
		lines_.clear();

	//cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);
	HoughLines(image_, lines_, 1, degree, threshold, 0, 0);

	//std::cout << "doVerticalHough # lines : " << lines.size();

	if (lines_.empty())
		return -1;

	allLines_.clear();
	allLines_.reserve(lines_.size());

	for (auto& line : lines_) {
		auto theta = line[1];
		if (theta <= degree * (180 - angleLimit_) && theta >= degree * angleLimit_)
			continue;
		auto p = computePointPair(line);
		allLines_.emplace_back(LineV(line, p));
	}

	if (allLines_.empty())
		return -2;
	//cerr << "FATAL ERROR, NO VERTICAL LINES DETECTED!";

	//drawLines(allLines, cv::Scalar(255, 0, 255));
	computeMeta();

	return 0;

}

inline tg::linePair HoughLinesR::computePointPair(cv::Vec2f& line) const {
	auto rho = line[0];
	auto theta = line[1];
	double a = cos(theta);
	double b = sin(theta);
	auto x0 = a * rho;
	auto y0 = b * rho;
	cv::Point2f pt1(static_cast<float>(x0 + 1000 * (-b)), static_cast<float>(y0 + 1000 * (a)));
	cv::Point2f pt2(static_cast<float>(x0 - 1000 * (-b)), static_cast<float>(y0 - 1000 * (a)));
	return tg::linePair(pt1, pt2);
}

inline tg::linePair HoughLinesR::computePointPair2(cv::Vec2f& line) const {
	auto rho = line[0];
	auto theta = line[1];

	// detect lower part of image
	if (rho < 0) {
		theta += 180.0f;
		rho = -rho;
	}

	// convert
	theta = static_cast<float>(theta * (1.0f / 180.0f) * CV_PI);

	auto imgCenterX = static_cast<unsigned int>(image_.cols) * 0.5f;
	auto imgCenterY = static_cast<unsigned int>(image_.rows) * 0.5f;

	double x0;
	double x1;
	double y0;
	double y1;

	if (line[1] != 0) {
		// non-vertical line
		x0 = -imgCenterX;
		x1 = imgCenterX;

		double cosTheta = cos(theta);
		double sinTheta = sin(theta);
		y0 = (-cosTheta * x0 + rho) / sinTheta;
		y1 = (-cosTheta * x1 + rho) / sinTheta;

	} else {
		// vertical line
		
		x0 = line[0];
		x1 = line[0];

		y0 = imgCenterY;
		y1 = -imgCenterY;

	}

	auto p0x = static_cast<float>(x0);
	auto p1x = static_cast<float>(x1);
	auto p0y = static_cast<float>(y0);
	auto p1y = static_cast<float>(y1);

	return tg::linePair(cv::Point2f(p0x, p0y), cv::Point2f(p1x, p1y));

}

inline void HoughLinesR::drawLines(vector<LineV>& line_pairs, cv::Scalar colour) {
	if (!showWindows_)
		return;

	if (line_pairs.empty())
		return;

	for (auto& lineV : line_pairs) {
		line(output_, lineV.points.first, lineV.points.second, colour, 1, CV_AA);
		//line(original, r.first, r.second, colour, 1, CV_AA);
	}
}

inline void HoughLinesR::drawLine(cv::Vec4d& line) {
	cv::Point2d p1(line[0], line[1]);
	cv::Point2d p2(line[2], line[3]);
	cv::line(output_, p1, p2, cv::Scalar(120, 120, 255), 2);
}

inline void HoughLinesR::showOutput() const {
	if (!showWindows_)
		return;

	imshow(windowName, output_);
}
