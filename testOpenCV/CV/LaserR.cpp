#include "LaserR.h"


bool LaserR::computeIntensityWeigth(cv::Mat& image, vector<v3<float>>& output) {

	// accu non-zero pixels.
	vector<cv::Point2i> nonZero;
	nonZero.reserve(image.cols * image.rows);

	cv::findNonZero(image, nonZero);

	// guard
	if (nonZero.empty()) return false;

	// clear if not empty
	if (!output.empty()) output.clear();

	// reserve enough space to avoid automatic resizing
	output.reserve(nonZero.size());

	configureXLine(image, nonZero, output);

	return !output.empty();

}