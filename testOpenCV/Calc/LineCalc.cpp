#include "LineCalc.h"
#include <opencv2/core.hpp>

#include "../namespaces/tg.h"

using namespace tg;

#ifdef CV_VERSION

/**
 * \brief Computes the intersection points based on horizontal line and two borders
 * \param horizontal_line The horizontal line
 * \param left_border The left border
 * \param right_border The right border
 * \param output The resulting intersection points (if any)
 * \return true if intersection points where computed, otherwise false
 */
bool LineCalc::computeIntersectionPoints(cv::Vec4d horizontal_line, const cv::Vec4d& left_border, const cv::Vec4d& right_border, cv::Vec4d& output) const {

	// align horizontal line
	horizontal_line[0] = left_border[0];
	horizontal_line[2] = right_border[2];

	cv::Point2d left_intersection;
	cv::Point2d right_intersection;

	// check intersection for both sides
	auto intersects_left = calc::intersection(left_border, horizontal_line, left_intersection);
	auto intersects_right = calc::intersection(right_border, horizontal_line, right_intersection);

	// check if there are two intersections
	if (!(intersects_left & intersects_right)) {
		log_time << "Error in intersection detection.\n";
		CV_Error(cv::Error::StsAssert, "Intersection points are invalid.");
		return false;
	}

	// assign result to output vector
	output[0] = left_intersection.x;
	output[1] = left_intersection.y;
	output[2] = right_intersection.x;
	output[3] = right_intersection.y;

	return true;

}

/**
 * \brief Adjust the marking rectangle based on the intersection points and specified buffer
 * \param marking_rect The rectangle to adjust
 * \param intersection_points The intersection points
 * \param buffer The buffer to apply
 */
void LineCalc::adjustMarkingRect(cv::Rect2d& marking_rect, cv::Vec4d& intersection_points, double buffer) {

	if (marking_rect.x == 0)
		CV_Error(cv::Error::BadROISize, "Marking rectangle start position is zero.");		
	else if (marking_rect.width == 0)
		CV_Error(cv::Error::BadROISize, "Marking rectangle width is zero.");

	marking_rect.x = intersection_points[0] + buffer;
	marking_rect.width = intersection_points[2] - marking_rect.x - buffer;

}

/**
 * \brief Adjusts the baselines according to intersection points and specified buffer
 * \param base_lines The baseline
 * \param intersection_points The intersection points
 * \param buffer The buffer
 */
void LineCalc::adjustBaseLines(cv::Vec4d& base_lines, cv::Vec4d& intersection_points, double buffer) {
	
	base_lines[0] = intersection_points[0] - buffer;
	base_lines[2] = intersection_points[2] + buffer;

}

/**
 * \brief Computes the intensity centroid for each X in the Y direction.
 * This gives the weighted Y position based off the intensity levels across that single X column
 * \param image The image to perform the computation on
 * \param output The output vector of points
 * \param upper_limit The upper limit of the rectangular cut out
 * \param lower_limit The lower limit of the rectangular cut out
 * \return The avg of the computed Y value across the entirety of the image matrix with regards to cut offs
 */
double LineCalc::computeRealIntensityLine(cv::Mat& image, std::vector<cv::Point2d>& output, int upper_limit, int lower_limit) {

	// generate vector with all X
	if (!output.empty())
		output.clear();

	// grab sizes
	auto cols = image.cols;
	auto rows = image.rows;

	// reserve enough space for the output point vector
	output.reserve(cols);

	// cut out rectangle without any X value set
	cv::Rect2d cutOut(0.0, lower_limit, 1.0, upper_limit);

	for (auto i = 0; i < cols; i++) {

		auto x = static_cast<double>(static_cast<signed int>(i));

		// create a new point in the list with only X value set
		output.emplace_back(cv::Point2d(x, 0.0));

		// adjust cutOut rectangle for current position
		cutOut.x = x;

		// create a new matrix based of the settings 
		auto B = cv::Mat(image, cutOut);

		// perform moments on the matrix without treating it as a binary image (which it is NOT)
		auto m = cv::moments(B, false);

		// compute x & y
		//auto x = m.m10 / m.m00; // x is not used, so no need to calculate it
		auto y = m.m01 / m.m00;

		// only include values above 0.0 in y-pos
		if (y > 0.0) {
			output.back().y = y;			
		}
	}

	auto avg = 0.0;

	for (const auto& h : output)
		avg += h.y;

	avg /= output.size();

	return avg;

}

#endif
