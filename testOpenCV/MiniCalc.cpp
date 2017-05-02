#include "MiniCalc.h"
#include "Util.h"
#include "Specs.h"

MiniCalc::MiniCalc() {
}

MiniCalc::~MiniCalc() {
}

void MiniCalc::SetMean(cv::Point2d newMean) {
	mean_ = newMean;
}

cv::Point2d MiniCalc::GetMean() const {
	return mean_;
}

void MiniCalc::AddMean(cv::Point2d meanToAdd) {
	mean_ += meanToAdd;
	mean_ /= 2;
}

void MiniCalc::SetVariance(cv::Point2d newVariance) {
	variance_ = newVariance;
}

cv::Point2d MiniCalc::GetVariance() const {
	return variance_;
}

void MiniCalc::AddVariance(cv::Point2d varianceToAdd) {
	variance_ += varianceToAdd;
	variance_ /= 2;
}

cv::Point2d MiniCalc::mean(vector<cv::Point>& pixels) {
	double resY = 0.0;
	double resX = 0.0;

	if (pixels.empty())
		return cv::Point2d(0.0, 0.0);

	for (auto& pixel_point : pixels) {
		resY += pixel_point.y;
		resX += pixel_point.x;
	}

	return cv::Point2d(resX / pixels.size(), resY / pixels.size());
}

double MiniCalc::mean(vector<double>& vec) {
	auto sum = 0.0;
	if (vec.empty())
		return sum;
	for (auto& v : vec)
		sum += v;
	return sum / vec.size();
}

double MiniCalc::meanX(vector<cv::Point>& pixels) {
	auto resX = 0.0;

	if (pixels.empty())
		return resX;

	for (auto& pixel_point : pixels)
		resX += pixel_point.x;

	return resX / pixels.size();
}

double MiniCalc::meanY(vector<cv::Point>& pixels) {
	auto resY = 0.0;

	if (pixels.empty())
		return resY;

	for (auto& pixel_point : pixels)
		resY += pixel_point.y;

	return resY / pixels.size();
}

cv::Point MiniCalc::quantileX(Quantile quant, vector<cv::Point>& pixels) {
	if (pixels.empty())
		return cv::Point(0, 0);

	if (quant == Quantile::Q0) return pixels.front();
	if (quant == Quantile::Q100) return pixels.back();
	auto index = quantileMap_[quant] * pixels.size();
	auto pixSorted(pixels);
	sort(pixSorted.begin(), pixSorted.end(), sortX);
	return pixSorted.at(static_cast<int>(index));
}

cv::Point MiniCalc::quantileY(Quantile quant, vector<cv::Point>& pixels) {
	if (quant == Quantile::Q0) return pixels.front();
	if (quant == Quantile::Q100) return pixels.back();
	auto index = quantileMap_[quant] * pixels.size();
	auto pixSorted(pixels);
	sort(pixSorted.begin(), pixSorted.end(), sortY);
	return pixSorted.at(static_cast<int>(index));
}

cv::Point MiniCalc::percentileX(double percentage, vector<cv::Point>& pixels) const {
	if (percentage == 0.0) return pixels.front();
	if (percentage == 1.0) return pixels.back();
	auto index = percentage * pixels.size();
	auto pixSorted(pixels);
	sort(pixSorted.begin(), pixSorted.end(), sortX);
	return pixSorted.at(static_cast<int>(index));
}

cv::Point MiniCalc::percentileY(double percentage, vector<cv::Point>& pixels) const {
	if (percentage == 0.0) return pixels.front();
	if (percentage == 1.0) return pixels.back();
	auto index = percentage * pixels.size();
	auto pixSorted(pixels);
	sort(pixSorted.begin(), pixSorted.end(), sortY);
	return pixSorted.at(static_cast<int>(index));
}

cv::Point2d MiniCalc::variance(vector<cv::Point>& pixels) const {
	if (pixels.empty())
		return cv::Point2d(0.0, 0.0);

	auto meanXY = mean(pixels);
	return variance(meanXY, pixels);
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

void MiniCalc::varianceAdd(vector<cv::Point>& pixels) {
	if (pixels.empty())
		return;

	auto newMean = mean(pixels);
}

double MiniCalc::varianceX(vector<cv::Point>& pixels) const {
	if (pixels.empty())
		return 0.0;

	auto mean = meanX(pixels);
	return varianceX(mean, pixels);
}

double MiniCalc::varianceX(double mean, vector<cv::Point>& pixels) const {
	if (pixels.empty())
		return 0.0;

	auto variance = 0.0;
	for (auto& pixel_point : pixels)
		variance += (1.0 / (pixels.size() - 1)) * pow(pixel_point.x - mean, 2.0);

	return variance;
}

double MiniCalc::varianceY(vector<cv::Point>& pixels) const {
	if (pixels.empty())
		return 0.0;

	auto mean = meanY(pixels);
	return varianceY(mean, pixels);
}

double MiniCalc::varianceY(double mean, vector<cv::Point>& pixels) const {
	if (pixels.empty())
		return 0.0;

	auto variance = 0.0;

	for (auto& pixel_point : pixels)
		variance += (1.0 / (pixels.size() - 1)) * pow(pixel_point.y - mean, 2.0);

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
		auto r = boundingRect(contours.at(i));
		if (/* halfImageY < r.y && */ r.y > highestRect.y) {
			highestRect = r;
		}
		//cout << r << endl;
	}

	return highestRect.y;
}

bool MiniCalc::saveData(string filename) const {
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	if (!fs.isOpened()) {
		cerr << "Error while opening " << filename << " for output." << endl;
		return false;
	}

	fs << "Calculation_Time" << Util::getTime();

	fs << "Mean" << mean_;
	fs << "Variance" << variance_;
	fs << "PlanarPixels" << m_PlanarPixels;

	fs.release();

	return true;
}

bool MiniCalc::generatePlanarPixels(cv::Mat& input, cv::Mat& output, vector<cv::Point>& pixels, vector<cv::Point2d>& gradientPixels) const {

	vector<cv::Point> pix;

	pix.reserve(input.cols);

	findNonZero(input, pix);

	gradientPixels.clear();

	// sort the list in X
	sort(pix.begin(), pix.end(), sortX);

	auto x = pix.front().x;
	auto count = 0;
	auto ySum = 0;
	auto totalYMean = 0.0;
	auto gradientSum = 0.0;
	//unsigned char value = image.at<unsigned char>(y, x);

	for (auto& p : pix) {
		if (p.x != x) {
			if (count > 0) {
				pixels.push_back(cv::Point(x, static_cast<int>(round(ySum / static_cast<double>(count)))));
				auto gradient = static_cast<unsigned char>(round(gradientSum / count));
				output.at<unsigned char>(pixels.back()) = gradient;
				gradientPixels.push_back(cv::Point2d(static_cast<double>(x), gradient));
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

	// remove any weird stuff
	for (auto& p : pixels) {
		if (p.y > totalYMean + 100) {

		}
	}


	return true;
}

uchar MiniCalc::getGradientYValues(cv::Mat& image, int x, int y, int maxY, int minY) {

	cv::Scalar colour;
	auto r = cv::Rect(cv::Point(x, y), cv::Size(1, 1));


	return 1;
}


double MiniCalc::calculatePixelToMm(double pixelLocation) {
	Specs s;
	return s.PixMm() * pixelLocation;
}

bool MiniCalc::computeDiags(cv::Mat& image, vector<cv::Mat>& diags) {

	// reserve space
	diags.clear();
	diags.reserve(image.cols * image.rows);

	// sorted by: positive -> negative

	// grab the positive diags
	for (auto i = 1; i < image.rows; ++i) {
		diags.push_back(image.diag(i));
	}

	for (auto i = image.rows; i >= 0; --i) {
		diags.push_back(image.diag(i));
	}

	return diags.empty() ^ true;
}

bool MiniCalc::computeDiagAvg(vector<cv::Mat>& diagonals, vd& output) {

	output.clear();
	output.reserve(diagonals.front().cols);


	for (auto& d : diagonals) {
		auto input = static_cast<unsigned char*>(d.data);
		for (auto j = 0; j < d.rows; j++) {
			auto sum = 0.0;
			for (auto i = 0; i < d.cols; i++)
				sum += input[d.step * j + i];
			if (sum > 0.0) {
				output.push_back(cv::Point2d(j, sum / d.cols));
				//cout << "sum : " << sum << endl;
			}
		}
	}

	return output.empty() ^ true;

}
