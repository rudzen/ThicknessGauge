#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/ring_buffer.hpp>
#include <opencv2/highgui.hpp>


class HistoPeak {

	int histSize = 256;

	struct Length {
		int pos1;
		int pos2;

		int size() const {
			return pos2 - pos1 + 1;
		}
	};

	struct PeakInfo {
		int pos;
		int left_size;
		int right_size;
		float value;
	};

	static PeakInfo peakInfo(int pos, int left_size, int right_size, float value) {
		PeakInfo output;
		output.pos = pos;
		output.left_size = left_size;
		output.right_size = right_size;
		output.value = value;
		return output;
	}

	cv::Mat histImage;

	std::vector<int> peaks;

public:

	void processImage(cv::Mat& image);

	void drawPeaks(cv::Mat& histImage, std::vector<int>& peaks) const;

	cv::Mat drawHistogram(cv::Mat& hist, int hist_h = 400, int hist_w = 1024, int hist_size = 256, cv::Scalar color = cv::Scalar(255, 255, 255), int type = 2) const;

	static std::vector<PeakInfo> findPeaks(cv::InputArray _src, int window_size);

	//if you play with the peak_per attribute value, you can increase/decrease the number of peaks found
	std::vector<int> getLocalMaximum(cv::InputArray _src, int smooth_size = 9, int neighbor_size = 3, float peak_per = 0.5) const;


};

inline void HistoPeak::processImage(cv::Mat& image) {
	float range[] = {0, 256};

	const float* histRange = {range};
	auto uniform = true;
	auto accumulate = false;

	cv::calcHist(&image, 1, nullptr, cv::Mat(), histImage, 1, &histSize, &histRange, uniform, accumulate);

	peaks = getLocalMaximum(histImage);
}

inline void HistoPeak::drawPeaks(cv::Mat& histImage, std::vector<int>& peaks) const {
	auto bin_w = cvRound(static_cast<double>(histImage.cols) / histSize);
	auto col = cv::Scalar(255, 0, 255);
	for (size_t i = 0; i < peaks.size(); i++)
		line(histImage, cv::Point(bin_w * peaks[i], histImage.rows), cv::Point(bin_w * peaks[i], 0), col);


	imshow("Peaks", histImage);
	//return EXIT_SUCCESS;
}

inline cv::Mat HistoPeak::drawHistogram(cv::Mat& hist, int hist_h, int hist_w, int hist_size, cv::Scalar color, int type) const {
	auto bin_w = cvRound(static_cast<double>(hist_w) / hist_size);

	cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

	/// Normalize the result to [ 0, histImage.rows ]
	normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

	switch (type) {
	case 1:
		for (auto i = 0; i < histImage.cols; i++) {
			const unsigned x = i;
			const unsigned y = hist_h;

			line(histImage, cv::Point(bin_w * x, y), cv::Point(bin_w * x, y - cvRound(hist.at<float>(i))), color);
		}

		break;
	case 2:
		for (auto i = 1; i < hist_size; ++i) {
			auto pt1 = cv::Point(bin_w * (i - 1), hist_h);
			auto pt2 = cv::Point(bin_w * i, hist_h);
			auto pt3 = cv::Point(bin_w * i, hist_h - cvRound(hist.at<float>(i)));
			auto pt4 = cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1)));
			cv::Point pts[] = {pt1, pt2, pt3, pt4, pt1};

			fillConvexPoly(histImage, pts, 5, color);
		}
		break;
	default:
		for (auto i = 1; i < hist_size; ++i) {
			line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
			     cv::Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))), color, 1, 8, 0);
		}

		break;
	}

	cv::imshow("Histogram", histImage);

	return histImage;
}

inline std::vector<HistoPeak::PeakInfo> HistoPeak::findPeaks(cv::InputArray _src, int window_size) {
	auto src = _src.getMat();

	auto slope_mat = src.clone();

	// Transform initial matrix into 1channel, and 1 row matrix
	auto src2 = src.reshape(1, 1);

	auto size = window_size / 2;

	Length up_hill, down_hill;
	std::vector<PeakInfo> output;

	auto pre_state = 0;
	auto i = size;

	while (i < src2.cols - size) {
		auto cur_state = src2.at<float>(i + size) - src2.at<float>(i - size);

		if (cur_state > 0)
			cur_state = 2;
		else if (cur_state < 0)
			cur_state = 1;
		else cur_state = 0;

		// In case you want to check how the slope looks like
		slope_mat.at<float>(i) = cur_state;

		if (pre_state == 0 && cur_state == 2)
			up_hill.pos1 = i;
		else if (pre_state == 2 && cur_state == 1) {
			up_hill.pos2 = i - 1;
			down_hill.pos1 = i;
		}

		if ((pre_state == 1 && cur_state == 2) || (pre_state == 1 && cur_state == 0)) {
			down_hill.pos2 = i - 1;
			auto max_pos = up_hill.pos2;
			if (src2.at<float>(up_hill.pos2) < src2.at<float>(down_hill.pos1))
				max_pos = down_hill.pos1;

			auto peak_info = peakInfo(max_pos, up_hill.size(), down_hill.size(), src2.at<float>(max_pos));

			output.push_back(peak_info);
		}
		i++;
		pre_state = static_cast<int>(cur_state);
	}
	return output;
}

inline std::vector<int> HistoPeak::getLocalMaximum(cv::InputArray _src, int smooth_size, int neighbor_size, float peak_per) const {
	auto src = _src.getMat().clone();

	std::vector<int> output;
	cv::GaussianBlur(src, src, cv::Size(smooth_size, smooth_size), 0);
	auto peaks = findPeaks(src, neighbor_size);

	double min_val, max_val;
	cv::minMaxLoc(src, &min_val, &max_val);

	for (size_t i = 0; i < peaks.size(); i++) {
		if (peaks[i].value > max_val * peak_per && peaks[i].left_size >= 2 && peaks[i].right_size >= 2)
			output.push_back(peaks[i].pos);
	}

	auto histImg = drawHistogram(src);
	drawPeaks(histImg, output);
	return output;
}
