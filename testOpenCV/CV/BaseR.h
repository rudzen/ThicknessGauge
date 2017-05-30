#pragma once
#include <opencv2/core/cvdef.h>
#include <string>

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

template <class T>
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

	cv::Rect_<T> markingRect;

	int imageOffset = 0;

	const double degree = CV_PI * (1.0 / 180.0);

	std::string windowName;

	bool showWindows_ = false;

public:

	void setOriginal(const cv::Mat& original) { original_ = original; }

	cv::Mat& getImage() { return image_; }

	void setImage(const cv::Mat& image) { image_ = image; }

	const cv::Rect_<T>& getMarkingRect() const {
		return markingRect;
	}

	void setMarkingRect(const cv::Rect_<T>& markingRect) {
		this->markingRect = markingRect;
	}

	bool isShowWindows() const {
		return showWindows_;
	}

	void setShowWindows(bool showWindows) {
		showWindows_ = showWindows;
	}
};
