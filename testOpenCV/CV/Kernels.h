#pragma once
#include <opencv2/core/mat.hpp>

class Kernels {
public:

	cv::Mat lineHKernel = (cv::Mat_<char>(1, 5) << 3 , 3 , 0 , -3 , -3);

	cv::Mat speckKernel = (cv::Mat_<char>(3, 3) <<
		0 , 0 , 0 ,
		0 , 1 , 0 ,
		0 , 0 , 0);

	cv::Mat lineVKernel = (cv::Mat_<char>(4, 4) <<
		0 , 0 , 1 , 1 ,
		0 , 1 , 1 , 1 ,
		1 , 1 , 1 , 0 ,
		1 , 1 , 0 , 0
	);

};
