#pragma once
#include <opencv2/core/mat.hpp>
#include <array>

class Data {
public:
	explicit Data(const cv::Size size) : size(size)
		                                   , exposure(0) {
			auto half = size.width * (1.0 / 2.0);
			size_t lim = cvRound(ceil(half));
			auto limHalf = lim >> 1;
			auto limQuarter = limHalf >> 1;
			left.reserve(limQuarter);
			right.reserve(limQuarter);
			center.reserve(limHalf);
		}

		// the left side points for the line
		std::vector<cv::Point2d> left;

		// the right side points for the line
		std::vector<cv::Point2d> right;

		// the center points
		std::vector<cv::Point2d> center;

		// the marking rect determins where in the frame the marking is located.
		cv::Rect2f markingRect;

		// offsets, which is exactly the pieces missing between left/center and center/right
#ifdef CV_VERSION
		cv::Vec2f offsets;
#else
		std::array<float, 2> offsets;
#endif

		// the total size of the representataion, includes full height and both offsets
		cv::Size size;

		unsigned int exposure;
};
