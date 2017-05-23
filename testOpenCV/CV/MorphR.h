#pragma once
#include "BaseR.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class MorphR : public BaseR<float> {

	cv::Mat output_;

	cv::MorphTypes method_;

	int iterations_;

	bool showWindow_;

public:

	explicit MorphR(cv::MorphTypes method, const int iterations, const bool showWindow)
		: method_(method), iterations_(iterations), showWindow_(showWindow) {
		windowName = "MorphR";
	}

	const cv::Mat& getResult() const {
		return output_;
	}

	cv::MorphTypes getMethod() const {
		return method_;
	}

	void setMethod(cv::MorphTypes method) {
		method_ = method;
	}

	int getIterations() const {
		return iterations_;
	}

	void setIterations(int iterations) {
		iterations_ = iterations;
	}

	void doMorph() {
		cv::morphologyEx(image_, output_, method_, cv::Mat(), cv::Point(-1, -1), iterations_);
		if (showWindow_)
			show();
	}

private:

	void show() const {
		cv::imshow(windowName, output_);
	}


};
