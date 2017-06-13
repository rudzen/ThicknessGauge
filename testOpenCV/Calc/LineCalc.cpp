#include "LineCalc.h"
#include <opencv2/core.hpp>

#include "../namespaces/tg.h"

using namespace tg;

#ifdef CV_VERSION

/**
 * \brief Adjust the marking rectangle based on the intersection points and specified buffer
 * \param marking_rect The rectangle to adjust
 * \param intersection_points The intersection points
 * \param buffer The buffer to apply
 */
//void LineCalc::adjustMarkingRect(cv::Rect2d& marking_rect, cv::Vec4d& intersection_points, double buffer) {
//
//    if (marking_rect.x == 0)
//    CV_Error(cv::Error::BadROISize, "Marking rectangle start position is zero.");
//    else if (marking_rect.width == 0)
//    CV_Error(cv::Error::BadROISize, "Marking rectangle width is zero.");
//
//    marking_rect.x = intersection_points[0] + buffer;
//    marking_rect.width = intersection_points[2] - marking_rect.x - buffer;
//
//}

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
