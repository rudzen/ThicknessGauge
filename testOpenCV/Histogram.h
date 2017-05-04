#pragma once
#include <map>
#include <array>
#include <ostream>
#include <opencv2/core/mat.hpp>
#include "Util.h"
#include <opencv2/videostab/ring_buffer.hpp>

class Histogram {

	const int MAX = 256;

	std::array<int, 256> histogram_;

	cv::Mat histogramImage_;

	int hist_w = 2048;

	int hist_h = hist_w / 3 * 4;

	int bin_w = Util::round(static_cast<double>(hist_w) / 256);

	void normalizeHistogramImage(const int max) {
		//auto modifier = max * hist_w;
		for (auto i = 0; i < MAX; i += 4) {
			histogram_[i] = (static_cast<double>(histogram_[i]) / max) * hist_w;
			histogram_[i + 1] = (static_cast<double>(histogram_[i + 1]) / max) * hist_w;
			histogram_[i + 2] = (static_cast<double>(histogram_[i + 2]) / max) * hist_w;
			histogram_[i + 3] = (static_cast<double>(histogram_[i + 3]) / max) * hist_w;
		}
	}

protected:

	/**
	 * \brief Populate local histogram array from external map
	 * \param intensityMap The intensity map to populate from
	 */
	void populateHistogram(std::map<int, unsigned char>& intensityMap, bool createImage = false);

	/**
	 * \brief Reset all histogram data
	 */
	void nullify();

	void createHistogramImage();

public:

	Histogram() {

	}

	const cv::Mat& histogramImage() const {
		return histogramImage_;
	}

	void clearHistogramImage() {
		histogramImage_ = cv::Mat(hist_h, hist_w, CV_8UC1, cv::Scalar(255, 255, 255));
		//histogramImage_ = cv::Mat::zeros(histogramImage.rows, histogramImage.cols, CV_8UC1);
	}

	friend std::ostream& operator<<(std::ostream& os, const Histogram& obj) {
		const auto space = ' ';
		for (auto i = 0; i < obj.MAX; i += 4) {
			os << obj.histogram_[i] << space;
			os << obj.histogram_[i + 1] << space;
			os << obj.histogram_[i + 2] << space;
			os << obj.histogram_[i + 3] << space;
		}
		return os;
	}

};

inline void Histogram::populateHistogram(std::map<int, unsigned char>& intensityMap, bool createImage = false) {
	if (intensityMap.empty())
		return;

	for (auto& i : intensityMap)
		histogram_[i.second]++;

	if (createImage)
		createHistogramImage();

}

inline void Histogram::nullify() {
	for (auto i = 0; i < MAX; i += 4) {
		histogram_[i] = 0;
		histogram_[i + 1] = 0;
		histogram_[i + 2] = 0;
		histogram_[i + 3] = 0;
	}

	clearHistogramImage();
}

inline void Histogram::createHistogramImage() {

	// get max value
	auto max = histogram_[0];
	for (auto& i : histogram_)
		if (max < i)
			max = i;

	normalizeHistogramImage(max);

	for (auto i = 0; i < MAX; i++)
		line(histogramImage_, cv::Point(bin_w * (i), hist_h), cv::Point(bin_w * (i), hist_h - histogram_[i]), cv::Scalar(0, 0, 0), 1, cv::LINE_8, 0);
}
