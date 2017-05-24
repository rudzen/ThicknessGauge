#pragma once

class ThicknessGaugeData {
	
protected:

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
