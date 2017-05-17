#pragma once
#include <opencv2/core/cvdef.h>
#include <string>

class BaseR {

protected:

	/**
	* \brief The original un-modified image
	*/
	cv::Mat original_;

	/**
	* \brief The image to process
	*/
	cv::Mat image_;

	cv::Rect2f markingRect;

	int imageOffset = 0;

	const double degree = CV_PI * (1.0 / 180.0);

	string windowName;

public:

	void setOriginal(const cv::Mat& original) { original_ = original; }

	const cv::Mat& getImage() const { return image_; }

	void setImage(const cv::Mat& image) { image_ = image; }

	const cv::Rect2f& getMarkingRect() const {
		return markingRect;
	}

	void setMarkingRect(const cv::Rect2f& markingRect) {
		this->markingRect = markingRect;
	}
};
