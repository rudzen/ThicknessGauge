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

		for (auto i = col.cols; i--;) {
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

	void removePepperNoise(cv::Mat& mask) {
		// For simplicity, ignore the top & bottom row border.
		for (auto y = 2; y < mask.rows - 2; y++) {
			// Get access to each of the 5 rows near this pixel.
			auto pThis = mask.ptr(y);
			auto pUp1 = mask.ptr(y - 1);
			auto pUp2 = mask.ptr(y - 2);
			auto pDown1 = mask.ptr(y + 1);
			auto pDown2 = mask.ptr(y + 2);

			// For simplicity, ignore the left & right row border.
			pThis += 2;
			pUp1 += 2;
			pUp2 += 2;
			pDown1 += 2;
			pDown2 += 2;
			for (auto x = 2; x < mask.cols - 2; x++) {
				auto v = *pThis; // Get the current pixel value (either 0 or 255).
				// If the current pixel is black, but all the pixels on the 2-pixel-radius-border are white
				// (ie: it is a small island of black pixels, surrounded by white), then delete that island.
				if (v == 0) {
					auto allAbove = *(pUp2 - 2) && *(pUp2 - 1) && *(pUp2) && *(pUp2 + 1) && *(pUp2 + 2);
					auto allLeft = *(pUp1 - 2) && *(pThis - 2) && *(pDown1 - 2);
					auto allBelow = *(pDown2 - 2) && *(pDown2 - 1) && *(pDown2) && *(pDown2 + 1) && *(pDown2 + 2);
					auto allRight = *(pUp1 + 2) && *(pThis + 2) && *(pDown1 + 2);
					auto surroundings = allAbove && allLeft && allBelow && allRight;
					if (surroundings) {
						// Fill the whole 5x5 block as white. Since we know the 5x5 borders
						// are already white, just need to fill the 3x3 inner region.
						*(pUp1 - 1) = 0;
						*(pUp1 + 0) = 0;
						*(pUp1 + 1) = 0;
						*(pThis - 1) = 0;
						*(pThis + 0) = 0;
						*(pThis + 1) = 0;
						*(pDown1 - 1) = 0;
						*(pDown1 + 0) = 0;
						*(pDown1 + 1) = 0;
					}
					// Since we just covered the whole 5x5 block with white, we know the next 2 pixels
					// won't be black, so skip the next 2 pixels on the right.
					pThis += 2;
					pUp1 += 2;
					pUp2 += 2;
					pDown1 += 2;
					pDown2 += 2;
				}
				// Move to the next pixel.
				pThis++;
				pUp1++;
				pUp2++;
				pDown1++;
				pDown2++;
			}
		}
	}
};
