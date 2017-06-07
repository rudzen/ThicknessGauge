#pragma once

#include <array>

class ThicknessGaugeData {

protected:

	using Frames = struct Frames {
		std::vector<cv::Mat> frames;
		std::string exp_ext;
		int exp_ms;

		Frames(const std::string& expExt, int expMs)
			: exp_ext(expExt),
			  exp_ms(expMs) {
		}
	};

	std::array<int, 3> exposures = {5000, 20000, 40000};
	std::array<std::string, 3> expusures_short = {"_5k", "_20k", "_40k"};
	std::vector<std::unique_ptr<Frames>> frameset;

	cv::Vec4f gaugeLine_;

	bool gaugeLineSet_;

	std::vector<cv::Mat> nulls_;

	cv::Size imageSize_;

	void initFrames() {
		frameset.clear();
		frameset.reserve(3);
		for (auto i = 0; i < exposures.size(); i++) {
			auto fra = std::make_unique<Frames>(expusures_short[i], exposures[i]);
			frameset.emplace_back(std::move(fra));
		}
		frameset.shrink_to_fit();
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
