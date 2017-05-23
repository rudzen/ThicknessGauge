// Rudy Alex Kohn

#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include "tg.h"
#include <opencv2/videostab/ring_buffer.hpp>

class DrawHelper {

private:

	cv::Scalar colour_;

public:

	static void makeWindow(std::string name);

	static void removeWindow(std::string& name);
	static void removeWindow(const std::string& name);

	void showImage(std::string& windowName, cv::Mat& image) const;
	void showImage(const std::string& name, cv::Mat& image) const;

	void drawHorizontalLine(cv::Mat* image, uint pos) const {
		drawHorizontalLine(image, pos, colour_);
	}

	static void drawHorizontalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
		line(*image, cv::Point(0, image->rows - pos), cv::Point(image->cols, image->rows - pos), colour, 1, cv::LINE_AA);
	}

	void drawVerticalLine(cv::Mat* image, uint pos) const {
		drawVerticalLine(image, pos, colour_);
	}

	static void drawVerticalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
		line(*image, cv::Point(pos, 0), cv::Point(pos, image->rows), colour);
	}

	void drawCenterAxisLines(cv::Mat* image) const {
		drawHorizontalLine(image, image->rows >> 1, colour_);
		drawVerticalLine(image, image->cols >> 1, colour_);
		//drawCenterAxisLines(image, colour_);
	}

	static void drawCenterAxisLines(cv::Mat* image, cv::Scalar colour) {
		drawHorizontalLine(image, image->rows >> 1, colour);
		drawVerticalLine(image, image->cols >> 1, colour);
		//line(*image, cv::Point(0, image->rows >> 1), cv::Point(image->cols, image->rows >> 1), colour);
		//line(*image, cv::Point(image->cols >> 1, 0), cv::Point(image->cols >> 1, image->rows), colour);
	}

	void drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position) const {
		drawText(image, text, position, colour_);
	}

	static void drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position, cv::Scalar colour);

	const cv::Scalar& colour() const {
		return colour_;
	}

	void colour(const cv::Scalar& colour) {
		colour_ = colour;
	}
};
