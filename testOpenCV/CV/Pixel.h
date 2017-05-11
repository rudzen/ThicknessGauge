#pragma once
#include <opencv2/core/mat.hpp>
#include "Util/Vec.h"
#include <iostream>
#include "Calc/MiniCalc.h"

class Pixelz {
	
	cv::Point2i me_;

public:


	static int Pixelz::getAllPixelSum(cv::Mat& image) {

		auto sum = 0;
		auto uc_pixel = image.data;
		for (auto row = 0; row < image.rows; ++row) {
			uc_pixel = image.data + row * image.step;
			for (auto col = 0; col < image.cols; ++col) {
				int a = uc_pixel[0];
				int b = uc_pixel[1];
				int c = uc_pixel[2];
				sum += a + b + c;
				uc_pixel += 3;
			}
		}
		return sum;
	}

	static uchar Pixelz::getElementIntensity(cv::Mat& image, cv::Point& point) {
		return image.at<uchar>(point);
	}

	uchar Pixelz::getElementIntensity(cv::Mat& image, v2<int>& point) const {
		cv::Point p(point.x, point.y);
		return getElementIntensity(image, p);
	}

	static double Pixelz::getYPixelsAvg(cv::Mat& image, int x) {
		auto sum = 0;
		auto count = 0;
		auto col = image.col(x);
		auto uc_pixel = col.data;

		for (auto i = col.cols; i--; ) {
			sum += uc_pixel[0];
			cout << "sum is now : " << sum << "\n";
			count++;
			uc_pixel++;
		}

		return sum / count;
	}

	int Pixelz::getHighestYpixel(cv::Mat& image, int x, MiniCalc& miniCalc) const {
		auto highest = image.rows;
		vector<cv::Point> pix;
		findNonZero(image, pix);

		sort(pix.begin(), pix.end(), miniCalc.sortX);

		auto intensitySum = 0.0;
		auto yAvg = 0.0;

		vector<cv::Point> elements;

		// grab all elements from the specific col
		for (auto& p : pix) {
			if (p.x < x)
				continue;
			if (p.x > x)
				break;
			elements.push_back(p);
		}

		auto highestIntensity = 0;

		for (auto& e : elements) {
			auto intensity = getElementIntensity(image, e);
			intensitySum += intensity;
			if (intensity > highestIntensity)
				highest = e.y;
			yAvg += e.y;
		}

		//intensitySum /= elements.size();
		//yAvg /= elements.size();

		//cout << "Intensity avg/x " << intensitySum << "/" << x << endl;
		//cout << "yAvg/x " << yAvg << "/" << x << endl;


		return highest;
	}
};
