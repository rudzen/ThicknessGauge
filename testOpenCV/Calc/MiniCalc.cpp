#include "Calc/MiniCalc.h"
#include "Util/Util.h"

MiniCalc::MiniCalc() {
}

MiniCalc::~MiniCalc() {
}

cv::Point2d MiniCalc::variance(cv::Point2d& mean, vector<cv::Point>& pixels) const {
	if (mean.x < 0.0 || mean.y < 0.0 || pixels.empty())
		return cv::Point2d(0.0, 0.0);

	cv::Point2d variance;
	auto size = pixels.size();
	for (auto& pixel_point : pixels) {
		variance.x += (1.0 / (size - 1)) * pow(pixel_point.x - mean.x, 2.0);
		variance.y += (1.0 / (size - 1)) * pow(pixel_point.y - mean.y, 2.0);
	}

	return variance;
}

int MiniCalc::calculateHighLow(vector<cv::Point>& pixels) {
	for (auto& pixel_point : pixels) {
		//cout << pixel_point << endl;
	}

	return 0;
}

int MiniCalc::highestPixelInLine(cv::Mat& image) const {
	auto const halfImageY = image.size().height / 2;

	vector<vector<cv::Point>> contours;
	findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cv::Rect highestRect(0, 0, 0, 0);

	for (auto i = 0; i < contours.size(); ++i) {
		auto r = boundingRect(contours[i]);
		if (/* halfImageY < r.y && */ r.y > highestRect.y) {
			highestRect = r;
		}
		//cout << r << endl;
	}

	return highestRect.y;
}

bool MiniCalc::generatePlanarPixels(cv::Mat& input, cv::Mat& output, vector<cv::Point2f>& pixels, vector<cv::Point2f>& gradientPixels) const {

	vector<cv::Point> pix;

	pix.reserve(input.cols);

	findNonZero(input, pix);

	gradientPixels.clear();
	gradientPixels.reserve(input.cols);
	for (auto x = input.cols; x--;)
		gradientPixels.emplace_back(cv::Point2d(0.0, 0.0));

	pixels.clear();
	pixels.reserve(input.cols);

	// sort the list in X
	sort(pix.begin(), pix.end(), sortX);

	auto x = pix.front().x;
	auto count = 0;
	auto ySum = 0;
	auto totalYMean = 0.0;
	auto gradientSum = 0.0;

	for (auto& p : pix) {
		if (p.x != x) {
			if (count > 0) {
				pixels.emplace_back(cv::Point(x, static_cast<int>(cvRound(ySum / static_cast<double>(count)))));
				auto gradient = static_cast<unsigned char>(cvRound(gradientSum / count));
				output.at<unsigned char>(pixels.back()) = gradient;
				gradientPixels[x].y = gradient;
				count = 0;
			}
			ySum = 0;
		}
		x = p.x;
		ySum += p.y;
		totalYMean += p.y;
		gradientSum += input.at<unsigned char>(p);
		count++;
	}

	if (pixels.empty())
		return false;

	totalYMean /= pixels.size();

	return true;
}

bool MiniCalc::getActualPixels(cv::Mat& image, vi& output, int yLimit) {
	vi result;
	cv::findNonZero(image, result);
	yLimit = abs(image.rows - yLimit);
	for (auto& p : result) {
		if (p.y <= yLimit)
			output.emplace_back(p);
	}
	return !output.empty();
}

bool MiniCalc::getActualPixels(vi& pixels, vi& target, double yLimit, int imageHeight) {
	if (!target.empty())
		target.clear();

	yLimit = abs(imageHeight - yLimit);

	for (auto& p : pixels) {
		if (p.y <= yLimit)
			target.emplace_back(p);
	}

	return !target.empty();
}
