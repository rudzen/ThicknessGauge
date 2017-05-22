#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/ring_buffer.hpp>
#include "DrawHelpers.h"
#include "../tg.h"

void DrawHelpers::drawHorizontalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
	line(*image, cv::Point(0, image->rows - pos), cv::Point(image->cols, image->rows - pos), colour, 1, cv::LINE_AA);
}

void DrawHelpers::drawVerticalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
	line(*image, cv::Point(pos, 0), cv::Point(pos, image->rows), colour);
}

void DrawHelpers::drawCenterAxisLines(cv::Mat* image, cv::Scalar& colour) {
	line(*image, cv::Point(0, image->rows >> 1), cv::Point(image->cols, image->rows >> 1), colour);
	line(*image, cv::Point(image->cols >> 1, 0), cv::Point(image->cols >> 1, image->rows), colour);
}

void DrawHelpers::drawHorizontalLine(cv::Mat* image, uint pos) {
	drawHorizontalLine(image, pos, cv::Scalar(255, 255, 255));
}

void DrawHelpers::drawVerticalLine(cv::Mat* image, uint pos) {
	drawVerticalLine(image, pos, cv::Scalar(255, 255, 255));
}

void DrawHelpers::drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position) {
	cv::Point pos;
	switch (position) {
	case tg::TextDrawPosition::UpperLeft:
			pos.x = image->cols / 3;
			pos.y = image->rows >> 2;
			break;
		case tg::TextDrawPosition::UpperRight:
			pos.x = image->cols - image->cols / 3;
			pos.y = image->rows >> 2;
			break;
		case tg::TextDrawPosition::LowerLeft:
			pos.x = image->cols / 3;
			pos.y = image->rows - 3 * (image->rows >> 2);
			break;
		case tg::TextDrawPosition::LowerRight:
			pos.x = image->cols - image->cols / 3;
			pos.y = image->rows - (image->rows >> 2);
			break;
		default:
			// oh noes..
			break;
	}
	putText(*image, text, pos, 1, 1.0, cv::Scalar(255, 255, 255), 2);
}
