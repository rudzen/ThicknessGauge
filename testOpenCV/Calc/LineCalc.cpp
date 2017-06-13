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

#endif
