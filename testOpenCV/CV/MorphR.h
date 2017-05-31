#pragma once
#include "BaseR.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class MorphR : public BaseR {

	cv::Mat output_;

	cv::Mat structureElement_;

	cv::MorphTypes method_;

	cv::MorphShapes elementShape_;

	int iterations_;

	bool showWindow_;

public:

	explicit MorphR(cv::MorphTypes method, const int iterations, const bool showWindow)
		: method_(method), iterations_(iterations), showWindow_(showWindow) {
		structureElement_ = cv::Mat();
		windowName = "MorphR";
		elementShape_ = cv::MORPH_RECT;
	}

	const cv::Mat& getResult() const {
		return output_;
	}

	cv::MorphTypes getMethod() const {
		return method_;
	}

	cv::MorphShapes getElementShape() const {
		return elementShape_;
	}

	void setElementShape(cv::MorphShapes elementShape) {
		elementShape_ = elementShape;
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

	void generateStructureElement(int size) {
		structureElement_ = getStructuringElement(elementShape_, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
	}

	void resetStructureElement() {
		structureElement_ = cv::Mat();
	}

	void doMorph() {
		cv::morphologyEx(image_, output_, method_, structureElement_, cv::Point(-1, -1), iterations_);
		if (showWindow_)
			show();
	}

private:

	void show() const {
		cv::imshow(windowName, output_);
	}


};
