#pragma once
#include <opencv2/core/mat.hpp>
#include "Util/Vec.h"
#include <iostream>
#include "Calc/MiniCalc.h"

   /*
	|  __
	| /__\
	| X~~|			"The eternal code god
	|-\|//-.		 watches over this mess."
   /|`.|'.' \			- R.A.Kohn, 2017
  |,|.\~~ /||
  |:||   ';||
  ||||   | ||
  \ \|     |`.
  |\X|     | |
  | .'     |||
  | |   .  |||
  |||   |  `.| JS
  ||||  |   ||
  ||||  |   ||
  `+.__._._+*/

class Pixelz {

public:

	/**
	 * \brief Determine if a pixel is not zero
	 * \param image The image
	 * \param pixel The point of the pixel
	 * \return true if > 0 otherwise false
	 */
	template <typename T>
	static bool isPixel(cv::Mat& image, cv::Point_<T>& pixel) {
		if (pixel.x > image.cols)
			return false;

		if (pixel.y > image.rows)
			return false;

		return image.at<uchar>(pixel) > 0;
	}


	/**
	 * \brief Get avg intensity for parsed line (0-255)
	 * \param image The image to use as base for intensity computation
	 * \param vector The vector to check for
	 * \param connectivity Line connectivity, default = 8
	 * \param leftToRight Reverse direction, default = false
	 * \return The average intensity for the line, ranging 0-255 (uchar)
	 */
	static uchar Pixelz::getLineAvgIntensity(cv::Mat& image, cv::Vec4f& vector, int connectivity = 8, bool leftToRight = false) {

		cv::LineIterator it(image, cv::Point(cvRound(vector[0]), cvRound(vector[1])), cv::Point(cvRound(vector[2]), cvRound(vector[3])), connectivity, leftToRight);

		if (it.count == 0)
			return 0;

		auto sum = 0.0;
		for (auto i = 0; i < it.count; i++, ++it) {
			auto pt = it.pos();
			sum += getElementIntensity(image, pt);
		}
		
		sum /= it.count;

		return static_cast<uchar>(cvRound(sum));

	}

	/**
	 * \brief Get the intensity of a specific point on a specific image
	 * \param image The image to get the intensity from
	 * \param point The location of the point to get the intensity from
	 * \return The intensity ranging from 0 to 255 (uchar)
	 */
	static uchar Pixelz::getElementIntensity(cv::Mat& image, cv::Point& point) {
		return image.at<uchar>(point);
	}

	uchar Pixelz::getElementIntensity(cv::Mat& image, v2<int>& point) const {
		cv::Point p(point.x, point.y);
		return getElementIntensity(image, p);
	}

	/**
	 * \brief Get average intensity for all pixels located at parsed X
	 * \param image The image to base the computation on
	 * \param x The row for which to get the intensity from
	 * \return The average intensity level
	 */
	static double Pixelz::getYPixelsAvg(cv::Mat& image, int x) {
		auto sum = 0;
		auto count = 0;
		auto col = image.col(x);
		auto uc_pixel = col.data;

		for (auto i = col.cols; i--;) {
			if (uc_pixel[0] == 0)
				continue;

			sum += uc_pixel[0];
			//cout << "sum is now : " << sum << "\n";
			count++;
			uc_pixel++;
		}

		return sum / count;
	}

	/**
	 * \brief Retrieve the highest located pixel in a specific coloum
	 * \param image The image to use
	 * \param x The coloumn to get the highest pixel from
	 * \param miniCalc For sorting struct only
	 * \return The location of the highest pixel location
	 */
	int Pixelz::getHighestYpixel(cv::Mat& image, int x, MiniCalc& miniCalc) const {
		auto highest = image.rows;
		vector<cv::Point> pix;
		findNonZero(image, pix);

		sort(pix.begin(), pix.end(), miniCalc.pixelXsort<int>());

		auto intensitySum = 0.0;
		auto yAvg = 0.0;

		vector<cv::Point> elements;

		// grab all elements from the specific col
		for (auto& p : pix) {
			if (p.x < x)
				continue;
			if (p.x > x)
				break;
			elements.emplace_back(p);
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

	/**
	 * \brief Removed problematic 2 pixel radius black island pixels by filling with white
	 * The intent for this function is to be used after a binary threshold
	 * and to make certain algorithms work more smoothly as the islands of pixels will be more "whole".
	 * \param mask The image to use
	 */
	static void removePepperNoise(cv::Mat& mask) {
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
						*(pUp1 - 1) = 255;
						*(pUp1 + 0) = 255;
						*(pUp1 + 1) = 255;
						*(pThis - 1) = 255;
						*(pThis + 0) = 255;
						*(pThis + 1) = 255;
						*(pDown1 - 1) = 255;
						*(pDown1 + 0) = 255;
						*(pDown1 + 1) = 255;
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
