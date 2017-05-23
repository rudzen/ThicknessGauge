#include "DrawHelper.h"
#include <opencv2/highgui/highgui.hpp>

void DrawHelper::makeWindow(std::string name) {
	cv::namedWindow(name, cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED);
}

void DrawHelper::removeWindow(std::string& name) {
	cv::destroyWindow(name);
	cv::waitKey(1);
}

void DrawHelper::removeWindow(const std::string& name) {
	cv::destroyWindow(name);
	cv::waitKey(1);
}

void DrawHelper::removeAllWindows() {
	cv::destroyAllWindows();
}

void DrawHelper::showImage(std::string& name, cv::Mat& image) const {
	if (!showWindows_)
		return;
	cv::imshow(name, image);
}

void DrawHelper::showImage(const std::string& name, cv::Mat& image) const {
	if (!showWindows_)
		return;
	cv::imshow(name, image);
}

void DrawHelper::drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position, cv::Scalar colour) {
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
	putText(*image, text, pos, 1, 1.0, colour, 2);
}
