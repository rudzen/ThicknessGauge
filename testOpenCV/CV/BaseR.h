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

	int imageOffset = 0;

	const double degree = CV_PI * (1.0 / 180.0);

	string windowName;

public:

	void setOriginal(const cv::Mat& original) { original_ = original; }

	const cv::Mat& getImage() const { return image_; }

	void setImage(const cv::Mat& image) { image_ = image; }

	void setImage(const cv::Mat& image, bool lowerHalf) {
		if (!lowerHalf) {
			setImage(image);
			return;
		}
		imageOffset = image.rows / 2;
		cv::Rect roi(0, 0, image.cols, imageOffset);
		image_ = image(roi);

	}

};
