#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"
#include "../tg.h"
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



using namespace tg;
using namespace std;

class HoughLinesR : public BaseR<float> {

public:

	typedef struct LineV {
		cv::Vec2f entry;
		linePair points;
		std::vector<cv::Point_<float>> elements;

		float slobe;

		LineV() {
			slobe = 0.0f;
		}

		LineV(cv::Vec2f entry, linePair points)
			: entry(entry),
			points(points) {
			elements.reserve(cvRound(Util::dist_manhattan(points.first.x, points.second.x, points.first.y, points.second.y)));
			LineV();
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
	cv::Vec2f leftBorder_;

	// the x coordinates of the right "border" of the marking
	cv::Vec2f rightBorder_;

	double leftY_ = 0.0;

	/* only used for UI display through open cv*/

	int rho_;

	int theta_;

	double angle_;

	int threshold;

	const int aperMin = 3;
	const int aperMax = 7;

	double srn_;

	double stn_;

	double minTheta_;

	double maxTheta_;

	int iAngleLimit_;

	double angleLimit_;

	bool showWindow_;

public:

	HoughLinesR(const int rho, const int theta, const int threshold, const bool showWindow)
		: rho_(rho),
		  theta_(theta),
		  threshold(threshold),
		  showWindow_(showWindow)
	{
		angle_ = CV_PI / 180 * theta;
		srn_ = 0;
		stn_ = 0;
		minTheta_ = 0.0;
		maxTheta_ = CV_PI;
		angleLimit_ = 0;
		windowName = "HoughLines";
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

	linePair HoughLinesR::computePointPair(cv::Vec2f& line) const;

	void drawLines(vector<LineV>& linePairs, cv::Scalar colour);

	void showOutput() const;

	void computeMeta();

	static void computeRectFromLines(vector<LineV>& input, cv::Rect2f& output);

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

	void doVerticalHough();

	void alignLeftY(int frameCount) {
		leftY_ /= frameCount;
		cout << "Horizontal baseline aligned to : " << leftY_ << " y" << endl;
		line(output_, cv::Point(0, cvRound(leftY_)), cv::Point(output_.cols / 2, cvRound(leftY_)), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		imshow(windowName, output_);
	}

	void setAngleLimit(double angleLimit) {
		this->angleLimit_ = angleLimit;
	}

	void setOriginal(cv::Mat& original) {
		original_ = original;
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

	const cv::Vec2f& getLeftBorder() const {
		return leftBorder_;
	}

	const cv::Vec2f& getRightBorder() const {
		return rightBorder_;
	}

};

inline void HoughLinesR::computeRectFromLines(vector<LineV>& input, cv::Rect2f& output) {
	
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

	cv::Rect2f leftRoi(0.0f, 0.0f, 0.0f, static_cast<float>(image_.rows));
	cv::Rect2f rightRoi(0.0f, 0.0f, 0.0f, static_cast<float>(image_.rows));

	computeRectFromLines(leftLines_, leftRoi);
	computeRectFromLines(rightLines_, rightRoi);

	markingRect.x = leftRoi.x;
	markingRect.y = leftRoi.y;
	markingRect.width = rightRoi.x - leftRoi.x + rightRoi.width;
	markingRect.height = static_cast<float>(image_.rows);

	leftBorder_[0] = leftRoi.x;
	leftBorder_[1] = leftRoi.x + leftRoi.width;

	rightBorder_[0] = rightRoi.x;
	rightBorder_[1] = rightRoi.x + rightRoi.width;

	if (showWindow_) {
		drawLines(leftLines_, cv::Scalar(0, 255, 0));
		drawLines(rightLines_, cv::Scalar(0, 0, 255));
		cv::rectangle(output_, leftRoi, cv::Scalar(255, 255, 0), 3, CV_AA);
		cv::rectangle(output_, rightRoi, cv::Scalar(255, 255, 0), 3, CV_AA);
		cv::rectangle(output_, markingRect, cv::Scalar(255, 0, 0), 3, CV_AA);
		showOutput();
	}

}

inline void HoughLinesR::rhocb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setTheta(value);
	cout << "Hough rho : " << value << endl;
}

inline void HoughLinesR::thetacb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setTheta(value);
	cout << "Hough theta : " << value << endl;
}

inline void HoughLinesR::thresholdcb(int value, void* userData) {
	auto that = static_cast<HoughLinesR*>(userData);
	that->setThreshold(value);
	cout << "Hough threshold : " << value << endl;
}

inline void HoughLinesR::doVerticalHough() {

	if (!lines_.empty())
		lines_.clear();

	//cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);
	HoughLines(image_, lines_, 1, degree, threshold, 0, 0);

	//std::cout << "doVerticalHough # lines : " << lines.size();

	if (lines_.empty())
		return;

	allLines_.clear();
	allLines_.reserve(lines_.size());

	for (auto& line : lines_) {
		auto theta = line[1];
		if (theta <= degree * (180 - angleLimit_) && theta >= degree * angleLimit_)
			continue;
		auto p = computePointPair(line);
		allLines_.push_back(LineV(line, p));
		//std::cout << "vert : " << p.first << " " << p.second << endl;
	}

	if (allLines_.empty())
		cerr << "FATAL ERROR, NO VERTICAL LINES DETECTED!";

	//drawLines(allLines, cv::Scalar(255, 0, 255));
	computeMeta();

}

inline void HoughLinesR::computeMeta() {
	
	if (allLines_.empty())
		return;

	auto size = allLines_.size();

	rightLines_.clear();
	rightLines_.reserve(size);

	leftLines_.clear();
	leftLines_.reserve(size);

	auto center = image_.cols / 2;

	for (auto& a : allLines_) {
		a.slobe = (a.entry[3] - a.entry[1]) / (a.entry[2] - a.entry[0]);
		if (a.points.first.x < center)
			leftLines_.push_back(a);
		else
			rightLines_.push_back(a);
	}

	auto lSize = leftLines_.size();
	auto rSize = rightLines_.size();

	if (lSize + rSize == 0)
		throw NoLineDetectedException("No marking lines detected.");

	if (lSize == 0)
		throw NoLineDetectedException("No marking left line detected.");

	if (rSize == 0)
		throw NoLineDetectedException("No marking right line detected.");

	for (auto& left : leftLines_) {
		cv::LineIterator it(image_, left.points.first, left.points.second, 8);
		left.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++, ++it)
			left.elements.push_back(it.pos());
	}

	if (lSize > 1)
		sort(leftLines_.begin(), leftLines_.end(), lineVsizeSort);

	for (auto& right : rightLines_) {
		cv::LineIterator it(image_, right.points.first, right.points.second, 8);
		right.elements.reserve(it.count);
		for (auto i = 0; i < it.count; i++, ++it)
			right.elements.push_back(it.pos());
	}

	if (rSize > 1)
		sort(rightLines_.begin(), rightLines_.end(), lineVsizeSort);

}

inline linePair HoughLinesR::computePointPair(cv::Vec2f& line) const {
	auto rho = line[0];
	auto theta = line[1];
	double a = cos(theta);
	double b = sin(theta);
	auto x0 = a * rho;
	auto y0 = b * rho;
	cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
	cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
	return linePair(pt1, pt2);
}

inline void HoughLinesR::drawLines(vector<LineV>& linePairs, cv::Scalar colour) {
	if (!showWindow_)
		return;

	if (linePairs.empty())
		return;

	for (auto& lineV : linePairs) {
		line(output_, lineV.points.first, lineV.points.second, colour, 1, CV_AA);
		//line(original, r.first, r.second, colour, 1, CV_AA);
	}
}

inline void HoughLinesR::showOutput() const {
	if (!showWindow_)
		return;

	imshow(windowName, output_);
}
