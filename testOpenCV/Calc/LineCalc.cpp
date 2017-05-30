#include "LineCalc.h"
#include <opencv2/core.hpp>

#ifdef CV_VERSION

/**
 * \brief Computes intersection points based on 4 points (2 x 2)
 * \param o1 Point one of pair one
 * \param p1 Point one of pair two
 * \param o2 Point two of pair one
 * \param p2 Point two of pair two
 * \param r The resulting point of intersection
 * \return true if intersection was found, otherwise false
 */
bool LineCalc::intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f& r) const {

	auto d1 = p1 - o1;
	auto d2 = p2 - o2;

	auto cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8) return false;

	auto x = o2 - o1;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

/** @overload
 * \param border The border vector containing x/y coordinates for both borders
 * \param line The line vector The line for which to check intersections with borders
 * \param result The resulting point coordinates if they intersect
 * \return true if intersection was found, otherwise false
 */
bool LineCalc::intersection(const cv::Vec4f& border, cv::Vec4f& line, cv::Point2f& result) const {
	return intersection(cv::Point2f(border[0], border[1]), cv::Point2f(line[0], line[1]), cv::Point2f(border[2], border[3]), cv::Point2f(line[2], line[3]), result);
}

/**
 * \brief Computes the intersection points based on horizontal line and two borders
 * \param horizontalLine The horizontal line
 * \param leftBorder The left border
 * \param rightBorder The right border
 * \param output The resulting intersection points (if any)
 * \return true if intersection points where computed, otherwise false
 */
bool LineCalc::computeIntersectionPoints(cv::Vec4f horizontalLine, const cv::Vec4f& leftBorder, const cv::Vec4f& rightBorder, cv::Vec4d& output) const {

	cv::Point2f leftIntersection;
	cv::Point2f rightIntersection;

	// align horizontal line
	horizontalLine[0] = leftBorder[0];
	horizontalLine[2] = rightBorder[2];

	// check intersection for both sides
	auto intersectsLeft = intersection(leftBorder, horizontalLine, leftIntersection);
	auto intersectsRight = intersection(rightBorder, horizontalLine, rightIntersection);

	// check if there are two intersections
	if (!(intersectsLeft & intersectsRight)) {
		Util::loge("Error in intersection detection.");
		CV_Error(cv::Error::StsAssert, "Intersection points are invalid.");
		return false;
	}

	// assign result to output vector
	output[0] = leftIntersection.x;
	output[1] = leftIntersection.y;
	output[2] = rightIntersection.x;
	output[3] = rightIntersection.y;

	return true;

}

/**
 * \brief Adjust the marking rectangle based on the intersection points and specified buffer
 * \param markingRect The rectangle to adjust
 * \param intersectionPoints The intersection points
 * \param buffer The buffer to apply
 */
void LineCalc::adjustMarkingRect(cv::Rect2d& markingRect, cv::Vec4d& intersectionPoints, double buffer) {

	if (markingRect.x == 0)
		CV_Error(cv::Error::BadROISize, "Marketing rectangle start position is zero.");		
	else if (markingRect.width == 0)
		CV_Error(cv::Error::BadROISize, "Marketing rectangle width is zero.");

	markingRect.x = intersectionPoints[0] + buffer;
	markingRect.width = intersectionPoints[2] - markingRect.x - buffer;

}

/**
 * \brief Adjusts the baselines according to intersection points and specified buffer
 * \param baseLines The baseline
 * \param intersectionPoints The intersection points
 * \param buffer The buffer
 */
void LineCalc::adjustBaseLines(cv::Vec4d& baseLines, cv::Vec4d& intersectionPoints, double buffer) {
	
	baseLines[0] = intersectionPoints[0] - buffer;
	baseLines[2] = intersectionPoints[2] + buffer;

}

/**
 * \brief Computes the intensity centroid for each X in the Y direction.
 * This gives the weighted Y position based off the intensity levels across that single X column
 * \param image The image to perform the computation on
 * \param output The output vector of points
 * \param upperLimit The upper limit of the rectangular cut out
 * \param lowerLimit The lower limit of the rectangular cut out
 * \param filename Filename for saving to text
 * \param fileOffsetY The offset for the values in the textfile
 * \return The avg of the computed Y value across the entirety of the image matrix with regards to cut offs
 */
double LineCalc::computeRealIntensityLine(cv::Mat& image, std::vector<cv::Point2d>& output, double upperLimit, double lowerLimit, std::string filename, double fileOffsetY) {

	// generate vector with all X
	if (!output.empty())
		output.clear();

	// grab sizes
	auto cols = image.cols;
	auto rows = image.rows;

	// reserve enough space for the output point vector
	output.reserve(cols * rows);

	// temporary matrix
	cv::Mat C;

	// cut out rectangle without any X value set
	cv::Rect2d cutOut(0.0, lowerLimit, 1.0, upperLimit);

	for (auto i = 0; i < cols; i++) {

		auto x = static_cast<double>(static_cast<signed int>(i));

		// create a new point in the list with only X value set
		output.emplace_back(cv::Point2d(x, 0.0));

		// adjust cutOut rectangle for current position
		cutOut.x = x;

		// create a new matrix based of the settings 
		auto B = cv::Mat(image, cutOut);

		// copy the rectanglular matrix to avoid cascade reference
		B.copyTo(C);

		// perform moments on the matrix without treating it as a binary image (which it is NOT)
		auto m = cv::moments(C, false);

		// compute x & y
		//auto x = m.m10 / m.m00; // x is not used, so no need to calculate it
		auto y = m.m01 / m.m00;

		// only include values above 0.0 in y-pos
		if (y > 0.0)
			output.back().y = y;
	}

	cv::Vec2d avg(0.0, 0.0);
	for (auto& h : output) {
	//	//out = to_string(h.y);
		avg[0] += h.y;
		avg[1] += rows - h.y;
	//	file << h.x << ' ' << (h.y + fileOffsetY) << '\n';
	//	fileY << h.y + fileOffsetY << '\n';
	//	fileButt << rows - (h.y + fileOffsetY) << '\n';
	}

	avg[0] /= output.size();
	avg[1] /= output.size();

	//fileY << "avg:" << avg[0] << '\n';
	//fileButt << "avg:" << avg[1] << '\n';

	//file.close();
	//fileY.close();
	//fileButt.close();

	return avg[1];

}

#endif
