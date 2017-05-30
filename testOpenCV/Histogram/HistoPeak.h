#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/ring_buffer.hpp>
#include <opencv2/highgui.hpp>
#include <ostream>

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
		int leftSize;
		int rightSize;
		float value;
	};

	static PeakInfo peakInfo(int pos, int leftSize, int rightSize, float value) {
		PeakInfo output;
		output.pos = pos;
		output.leftSize = leftSize;
		output.rightSize = rightSize;
		output.value = value;
		return output;
	}

	cv::Mat _org = cv::Mat::zeros(1, 1, CV_8S);

	//	cv::Mat& org_;

	cv::Mat histImage_;

	std::vector<int> peaks_;

	std::vector<int> dales_;

	std::vector<cv::Point2i> histElements_;

public:

	HistoPeak() /*:  org_(_org) */ {
	}

	const std::vector<cv::Point2i>& getHistElements() const {
		return histElements_;
	}

	const cv::Mat& histImage() const {
		return histImage_;
	}

	cv::Mat colourMeDirty(int lowerBoundry, int upperBoundry, cv::Scalar col) const;
	void processImage(cv::Mat& image, bool uniform, bool accumulate);

	void drawPeaks(cv::Mat& histImage, std::vector<int>& peaks) const;

	void drawDales(cv::Mat& histImage, std::vector<int>& dales) const;

	cv::Mat drawHistogram(cv::Mat& hist, int histH = 400, int histW = 2048, int histSize = 256, cv::Scalar color = cv::Scalar(255, 255, 255), int type = 2) const;

	static std::vector<PeakInfo> findPeaks(cv::InputArray _src, int window_size);

	static std::vector<PeakInfo> findDales(cv::InputArray _src, int windowSize);

	//if you play with the peak_per attribute value, you can increase/decrease the number of peaks found
	std::vector<int> getLocalMaximum(cv::InputArray _src, int smoothSize = 9, int neighborSize = 3, float peakPer = 0.1) const;

	std::vector<int> getLocalMinimum(cv::InputArray _src, int smoothSize = 9, int neighborSize = 3, float dalePer = 0.5) const;
	//std::vector<int> getLocalMinimum(cv::InputArray _src, int smoothSize = 9, int neighborSize = 3, float dalePer = 0.5) const;

	friend std::ostream& operator<<(std::ostream& os, const HistoPeak& obj) {
		os << "HistoPeak {";
		os << "peaks: ";
		for (auto& p : obj.peaks_)
			os << ' ' << p;

		os << "\ndales: ";
		for (auto& d : obj.dales_)
			os << ' ' << d;
		os << "}\n";
		return os;
	}
};

inline cv::Mat HistoPeak::colourMeDirty(int lowerBoundry, int upperBoundry, cv::Scalar col) const {

	vector<cv::Point2i> pix;

	findNonZero(_org, pix);

	cv::Mat img = cv::Mat::zeros(_org.rows, _org.cols, CV_32F);

	if (pix.empty())
		return img;

	cv::cvtColor(_org, img, cv::COLOR_GRAY2BGR);

	for (auto& p : pix) {
		auto intensity = _org.at<unsigned char>(p);
		if (intensity >= lowerBoundry && intensity <= upperBoundry)
			cv::line(img, p, p, col);
	}

	return img;
}

inline void HistoPeak::processImage(cv::Mat& image, bool uniform, bool accumulate) {

	_org = image.clone();

	auto minRange = 0.0;
	auto maxRange = 0.0;

	cv::minMaxLoc(image, &minRange, &maxRange);

	// default = 0, 256
	float range[] = {1, 255};

	const float* histRange = {range};

	cv::calcHist(&image, 1, nullptr, cv::Mat(), histImage_, 1, &histSize, &histRange, uniform, accumulate);

	peaks_ = getLocalMaximum(histImage_);
	dales_ = getLocalMinimum(histImage_);

	if (peaks_.empty())
		return;

	int low = 0;
	int high = histSize;
	int split = histSize / 2;

	auto size = peaks_.size();

	switch (size) {
	case 2:
		low = peaks_.front();
		high = peaks_.back();
		split = (low + high) / 2;
		break;
	case 1:


		break;
	default: ;
		//return;
	}

	auto lowest = histImage_.rows;
	for (auto i = 1; i < split; ++i) {
		auto current = histImage_.at<float>(i);
		if (current < lowest && current > 0) {
			lowest = Util::round(current);
		}
	}

	//cv::Mat dst, cdst;
	//cv::Canny(image, dst, 60, 90, 3);
	//cv::cvtColor(dst, cdst, CV_GRAY2BGR);
	///// Canny detector

	///// Using Canny's output as a mask, we display our result
	//dst = cv::Scalar::all(0);
	//vector<cv::Vec2f> lines;
	//auto baseAngle = CV_PI / 180;
	////HoughLines(dst, lines, 1, baseAngle, 20, 0, 0);
	//HoughLines(dst, lines, 1, baseAngle, 100);
	//cout << "#HoughLines : " << lines.size() << endl;
	////cdst = cv::Mat::zeros(image.rows, image.cols, image.type());
	//for (size_t i = 0; i < lines.size(); i++) {
	//	auto theta = lines[i][1];
	//	cout << theta << endl;
	//	//		if (theta>baseAngle * 170 || theta<baseAngle * 10) {
	//	auto rho = lines[i][0];
	//	double a = cos(theta);
	//	double b = sin(theta);
	//	auto x0 = a * rho;
	//	auto y0 = b * rho;
	//	cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
	//	cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
	//	line(cdst, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);
	//	//		}
	//}

	////vector<cv::Vec4i> lines;
	////HoughLinesP(dst, lines, 1, (CV_PI / 180) * 45.0, 50, 80, 5);
	////for (size_t i = 0; i < lines.size(); i++) {
	////	auto l = lines[i];
	////	line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
	////}
	//imshow("detected lines", cdst);

	imshow("colour me freaky   0-30", colourMeDirty(0, 30, cv::Scalar(0, 0, 255)));
	imshow("colour me freaky 31-100", colourMeDirty(31, 100, cv::Scalar(0, 0, 255)));
	imshow("colour me freaky 101-150", colourMeDirty(101, 150, cv::Scalar(255, 0, 255)));
	imshow("colour me freaky 151-200", colourMeDirty(151, 200, cv::Scalar(255, 0, 255)));
	imshow("colour me freaky 201-256", colourMeDirty(201, 256, cv::Scalar(0, 255, 255)));


}

inline void HistoPeak::drawPeaks(cv::Mat& histImage, std::vector<int>& peaks) const {
	auto bin_w = cvRound(static_cast<double>(histImage.cols) / histSize);
	auto col = cv::Scalar(255, 0, 255);
	for (auto& p : peaks)
		line(histImage, cv::Point(bin_w * p, histImage.rows), cv::Point(bin_w * p, 0), col, 2);

	imshow("Peaks", histImage);
}

inline void HistoPeak::drawDales(cv::Mat& histImage, std::vector<int>& dales) const {
	auto bin_w = cvRound(static_cast<double>(histImage.cols) / histSize);
	auto col = cv::Scalar(255, 255, 0);
	for (auto& d : dales)
		line(histImage, cv::Point(bin_w * d, histImage.rows), cv::Point(bin_w * d, 0), col, 2);

	imshow("Dales", histImage);
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
		for (auto i = 1; i < hist_size; ++i)
			line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
			     cv::Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))), color, 1, 8, 0);

		break;
	}

	//cv::flip(histImage, histImage, 1);

	cv::imshow("Histogram", histImage);

	return histImage;
}

inline std::vector<HistoPeak::PeakInfo> HistoPeak::findPeaks(cv::InputArray _src, int windowSize) {
	auto src = _src.getMat();

	auto slope_mat = src.clone();

	// Transform initial matrix into 1channel, and 1 row matrix
	auto src2 = src.reshape(1, 1);

	auto size = windowSize / 2;

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

			output.emplace_back(peak_info);
		}
		i++;
		pre_state = static_cast<int>(cur_state);
	}
	return output;
}

inline std::vector<HistoPeak::PeakInfo> HistoPeak::findDales(cv::InputArray _src, int windowSize) {
	auto src = _src.getMat();

	auto slope_mat = src.clone();

	// Transform initial matrix into 1channel, and 1 row matrix
	auto src2 = src.reshape(1, 1);

	auto size = windowSize / 2;

	Length up_hill;
	Length down_hill;

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
			down_hill.pos1 = i;
		else if (pre_state == 2 && cur_state == 1) {
			down_hill.pos2 = i - 1;
			up_hill.pos1 = i;
		}

		if ((pre_state == 1 && cur_state == 2) || (pre_state == 1 && cur_state == 0)) {
			up_hill.pos2 = i - 1;
			auto max_pos = down_hill.pos1;
			if (src2.at<float>(down_hill.pos1) < src2.at<float>(up_hill.pos2))
				max_pos = up_hill.pos2;

			auto peak_info = peakInfo(max_pos, up_hill.size(), down_hill.size(), src2.at<float>(max_pos));

			output.emplace_back(peak_info);
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

	double min_val;
	double max_val;

	cv::minMaxLoc(src, &min_val, &max_val);

	for (auto& p : peaks) {
		if (p.value > max_val * peak_per && p.leftSize >= 2 && p.rightSize >= 2)
			output.emplace_back(p.pos); // could be pos
	}

	auto histImg = drawHistogram(src);
	drawPeaks(histImg, output);
	return output;
}

inline std::vector<int> HistoPeak::getLocalMinimum(cv::InputArray _src, int smoothSize, int neighborSize, float dalePer) const {
	auto src = _src.getMat().clone();

	std::vector<int> output;
	cv::GaussianBlur(src, src, cv::Size(smoothSize, smoothSize), 0);
	auto dales = findDales(src, neighborSize);

	double min_val;
	double max_val;

	cv::minMaxLoc(src, &max_val, &min_val);

	for (auto& d : dales) {
		if (d.value > max_val * dalePer && d.leftSize >= 2 && d.rightSize >= 2)
			output.emplace_back(d.pos);
	}

	auto histImg = drawHistogram(src);
	drawDales(histImg, output);
	return output;
}
