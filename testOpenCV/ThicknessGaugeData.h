#pragma once

#include <array>

class ThicknessGaugeData {
	
protected:

	using Data = struct Data {

		Data(const cv::Size size, std::vector<cv::Point2f>& leftPoints, std::vector<cv::Point2f>& rightPoints, std::vector<cv::Point2f>& centerPoints) : left(leftPoints)
		                                                                                                                                               , right(rightPoints)
		                                                                                                                                               , center(centerPoints)
		                                                                                                                                               , size(size)
		                                                                                                                                               , exposure(0) {
			auto half = size.width * (1.0 / 2.0);
			size_t lim = cvRound(ceil(half));
			auto limHalf = lim >> 1;
			auto limQuarter = limHalf >> 1;
			left.reserve(limQuarter);
			right.reserve(limQuarter);
			center.reserve(limHalf);
		}

		// the left side points for the line
		std::vector<cv::Point2f>& left;

		// the right side points for the line
		std::vector<cv::Point2f>& right;

		// the center points
		std::vector<cv::Point2f>& center;

		// offsets, which is exactly the pieces missing between left/center and center/right
		std::array<float, 2> offsets;

		// the total size of the representataion, includes full height and both offsets
		cv::Size size;

		unsigned int exposure;
	};

	vector<cv::Mat> frames;

	LineConfig lineConfig_;

	cv::Vec4f gaugeLine_;

	double avgGaugeHeight_;

	bool gaugeLineSet_;

	std::vector<cv::Mat> nulls_;

	cv::Size imageSize_;

	int imageType_;

public:

	void setImageType(int imageType);

	int getImageType() const;

	void setImageSize(cv::Size size);

	void setImageSize(const int width, const int height);

public:

	ThicknessGaugeData(): avgGaugeHeight_(0), gaugeLineSet_(false), imageType_(0) {
	}

};

inline void ThicknessGaugeData::setImageType(int imageType) {
	imageType_ = imageType;
}

inline int ThicknessGaugeData::getImageType() const {
	return imageType_;
}

inline void ThicknessGaugeData::setImageSize(cv::Size size) {
	imageSize_ = size;
}

inline void ThicknessGaugeData::setImageSize(const int width, const int height) {
	imageSize_.width = width;
	imageSize_.height = height;
}
