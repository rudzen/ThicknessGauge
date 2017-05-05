#pragma once
#include <map>
#include <array>
#include <ostream>
#include <opencv2/core/mat.hpp>
#include "Util.h"
#include <opencv2/videostab/ring_buffer.hpp>

class Histogram {

	int valueMax_;

	const int histSize = 256; // bin size

	std::array<float, 256> histogram_;

	cv::Mat histogramImage_;

	cv::Mat hist_;
	cv::Mat histNormalized_;

	int hist_w = 1920;

	int hist_h = 1080;

	int bin_w = Util::round(static_cast<double>(hist_w) / histSize);

	vector<int> dale_;

private:

	void normalizeHistogramImage(const int max) {

		//auto modifier = max * hist_w;
		for (auto i = 0; i < histSize; ++i) {
			histogram_[i] = (static_cast<double>(histogram_[i]) / max) * hist_w;
		}
	}

public:

	Histogram(): valueMax_(0) {

	}

	/**
	 * \brief Populate local histogram array from external map
	 * \param intensityMap The intensity map to populate from
	 */
	void populateHistogram(std::map<int, unsigned char>& intensityMap, bool createImage);

	void populateHistogram(cv::Mat& image);
	void saveSimpleData(std::string& filename);

	/**
	 * \brief Reset all histogram data
	 */
	void nullify();

	void createHistogramImage();

	const cv::Mat& histogramImage() const {
		return histogramImage_;
	}

	void clearHistogramImage() {
		histogramImage_ = cv::Mat(hist_h, hist_w, CV_8UC1, cv::Scalar(0, 0, 0));
		//histogramImage_ = cv::Mat::zeros(histogramImage.rows, histogramImage.cols, CV_8UC1);
	}

	friend std::ostream& operator<<(std::ostream& os, const Histogram& obj) {
		const auto space = ' ';
		for (auto i = 0; i < obj.histSize; i += 4) {
			os << obj.histogram_[i] << space;
			os << obj.histogram_[i + 1] << space;
			os << obj.histogram_[i + 2] << space;
			os << obj.histogram_[i + 3] << space;
		}
		return os;
	}

};

inline void Histogram::populateHistogram(std::map<int, unsigned char>& intensityMap, bool createImage) {
	if (intensityMap.empty())
		return;

	for (auto& i : intensityMap)
		histogram_[i.second]++;

	if (createImage)
		createHistogramImage();

}

inline void Histogram::populateHistogram(cv::Mat& image) {
	
	if (image.empty())
		return;

	nullify();

	float range[] = { 0, 255 };
	const float* ranges[] = { range };

	calcHist(&image, 1, nullptr, cv::Mat(), hist_, 1, &histSize, ranges, true, false);

	clearHistogramImage();

	normalize(hist_, histNormalized_, 0, histogramImage_.rows, cv::NORM_MINMAX, -1, cv::Mat());

	for (auto i = 0; i < histSize; i++) {
		cv::line(histogramImage_, cv::Point(bin_w * (i - 1), hist_h - cvRound(histNormalized_.at<float>(i - 1))),
				 cv::Point(bin_w * (i), hist_h - cvRound(histNormalized_.at<float>(i))),
				 cv::Scalar(255, 0, 0), 1, cv::LINE_8, 0);
	}


	for (auto i = 0; i < histSize; ++i) {
		auto val = cvRound(hist_.at<float>(1, i));
		if (val > 0)
			dale_.push_back(val);
	}

	// attempt to seperate highest peak with line
	//auto currentValley = 0;
	//for (auto i = 2; i < hist_.rows - 2; ++i) {
	//	auto val = cvRound(hist_.at<float>(1, i));
	//	if (val < 500)
	//		continue;
	//	auto previous = cvRound(hist_.at<float>(1, i - 2));
	//	if (previous < 500)
	//		continue;
	//	auto next = cvRound(hist_.at<float>(1, i + 2));
	//	if (next < 500)
	//		continue;
	//	if (next > val && previous < val) {
	//		cv::line(histogramImage_, cv::Point(i, 0), cv::Point(i, hist_h), cv::Scalar(255, 255, 255));
	//	}
	//}

}


inline void Histogram::saveSimpleData(std::string& filename) {
	std::ofstream file(filename.c_str());
	
	for (auto& h : dale_) {
		file << h << '\n';
	}
	//for (auto h = 0; h < histSize; h++) {
	//	auto binVal = hist_.at<float>(h);
	//	if (h > 0)
	//		file << '\t';
	//	file << binVal;
	//}
	file.close();
}

inline void Histogram::nullify() {
	for (auto i = 0; i < histSize; i += 4) {
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

	const cv::Scalar black(0, 0, 0);

	for (auto i = 0; i < histSize; i++) {
		auto x = bin_w * i;
		line(histogramImage_, cv::Point(x, hist_h), cv::Point(x, hist_h - histogram_[i]), black, 1, cv::LINE_8, 0);
	}
}
