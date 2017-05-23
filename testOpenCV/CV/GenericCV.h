#pragma once
#include <opencv2/core.hpp>
#include <opencv2/videostab/inpainting.hpp>

// generic opencv helper functions (statics)

class GenericCV {
public:
	
	static void adaptiveThreshold(cv::Mat& input, cv::Mat& output, double* maxVal) {
		cv::adaptiveThreshold(input, output, *maxVal, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, 10);
	}


};
