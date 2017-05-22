#pragma once
#include "BaseR.h"
#include <opencv2/core/mat.hpp>
#include "tg.h"

class DrawHelpers {

public:

	/**
 * \brief Draws a horizontal line on the parsed image
 * \param image The image to draw the line on
 * \param pos The position in Y where the line should be drawn
 * \param colour The scalar colour of the line
 */
	static void drawHorizontalLine(cv::Mat* image, uint pos, cv::Scalar colour);

	static void drawVerticalLine(cv::Mat* image, uint pos, cv::Scalar colour);

	static void drawCenterAxisLines(cv::Mat* image, cv::Scalar& colour);

	static void drawHorizontalLine(cv::Mat* image, uint pos);

	static void drawVerticalLine(cv::Mat* image, uint pos);

	static void drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position);
};
