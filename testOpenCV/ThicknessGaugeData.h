#pragma once

#include <array>

//class Frames {
//public:
//	std::vector<cv::Mat> frames;
//	std::string exp_ext;
//	int exp_ms;
//};

class ThicknessGaugeData {

protected:

	std::array<int, 3> exposures = {5000, 20000, 40000};
	std::array<std::string, 3> expusures_short = {"_5k", "_20k", "_40k"};
	std::array<vector<cv::Mat>, 3> frames;

	//std::vector<Frames> frames;

	cv::Vec4f gaugeLine_;

	bool gaugeLineSet_;

	std::vector<cv::Mat> nulls_;

	cv::Size imageSize_;

	void initFrames() {
		for (auto i = 0; i < exposures.size(); i++) {
			//frames[i].exp_ext = expusures_short[i];
			//frames[i].exp_ms = exposures[i];
			//frames.emplace_back(expusures_short[i], exposures[i]);
		}

	}

public:

	ThicknessGaugeData() {
		initFrames();
	}

	void setImageSize(cv::Size size);

	void setImageSize(const int width, const int height);

};

inline void ThicknessGaugeData::setImageSize(cv::Size size) {
	imageSize_ = size;
}

inline void ThicknessGaugeData::setImageSize(const int width, const int height) {
	imageSize_.width = width;
	imageSize_.height = height;
}
