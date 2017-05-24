#pragma once

class ThicknessGaugeData {
	
protected:

	vector<cv::Mat> frames;

	// The pixels located in the image
	vi pixels_;
	vi allPixels_;
	vi measureLine_;
	vi rightSideLine_;
	vi leftSideLine_;
	vi intensity_;

	vector<v2<int>> lines_;
	v2<int> center_;

	LineConfig lineConfig_;

	cv::Vec4f gaugeLine_;

	double avgGaugeHeight_;

	bool gaugeLineSet_;

	std::vector<cv::Mat> nulls_;

protected:
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

	cv::Mat& GetPlanarImage();

	void setPlanarImage(const cv::Mat& mat);

	const vi& getPixels() const;
	const vi& getAllPixels() const;
	const vi& getMeasureLine() const;
	const vi& getRightSideLine() const;
	const vi& getLeftSideLine() const;
	const vi& getIntensity() const;

	double getIntensity(int x) const;

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

inline const vi& ThicknessGaugeData::getPixels() const {
	return pixels_;
}

inline const vi& ThicknessGaugeData::getAllPixels() const {
	return allPixels_;
}

inline const vi& ThicknessGaugeData::getMeasureLine() const {
	return measureLine_;
}

inline const vi& ThicknessGaugeData::getRightSideLine() const {
	return rightSideLine_;
}

inline const vi& ThicknessGaugeData::getLeftSideLine() const {
	return leftSideLine_;
}

inline const vi& ThicknessGaugeData::getIntensity() const {
	return intensity_;
}

inline double ThicknessGaugeData::getIntensity(int x) const {
	return intensity_.at(x).y;
}
