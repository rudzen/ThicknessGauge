#pragma once
#include <vector>
#include "PointData.h"
#include <map>
#include "Util.h"

/**
 * \brief Line class, contains information from a singular captured frame.
 * Including defined sections for 3 sides (right/center/left), frame reference and test output matrix.
 */
class Line {
	
	enum class SortMethod {
		none, x, y
	};

	struct elementYsort {
		bool operator()(cv::Point2d pt1, cv::Point2d pt2) const { return pt1.y < pt2.y; }
	} sortY;

	struct elementXsort {
		bool operator()(cv::Point2d pt1, cv::Point2d pt2) const { return pt1.x < pt2.x; }
	} sortX;

public:

	enum class Location {
		right, left, center
	};

private:

	const map<Location, int> locationMap = { { Location::right , 0 }, { Location::center, 1 }, { Location::left, 2 } };

	map<int, unsigned char> intensity_;

	/**
	 * \brief Differentiates points in a vector
	 * \param input The vector to be differentiated
	 * \param output The results
	 */
	static void differentiateY(std::vector<cv::Point2d>& input, std::vector<cv::Point2d>& output);

	/**
	 * \brief Combines two vectors into a third
	 * \param sourceOne The first vector
	 * \param sourceTwo The second vector
	 * \param target The target vector with sourceOne and sourceTwo data
	 * \param sortX Sorting method to be used when combining is done
	 */
	void combine(std::vector<cv::Point2d>& sourceOne, std::vector<cv::Point2d>& sourceTwo, std::vector<cv::Point2d> target, SortMethod sortX) const;

	/**
	 * \brief Get the pixel intensity of a location from the current frame
	 * \param location The point location to get the intensity from
	 * \return The grey scale pixel intensity
	 */
	unsigned char getPixelIntensity(cv::Point2d& location);

	/**
	 * \brief Generates the output matrix based on the current elements
	 */
	void generateOutput();

	/**
	 * \brief The frame for which the data set in the class is based
	 */
	cv::Mat frame_;

	/**
	 * \brief Matrix representation of the data vectors in the class
	 */
	cv::Mat output_;

	/**
	 * \brief All the sparse elements
	 */
	std::vector<cv::Point2d> allSparse_;

	/**
	 * \brief Left side of the elements
	 */
	std::vector<cv::Point2d> left_;

	/**
	 * \brief Right side of the elements
	 */
	std::vector<cv::Point2d> right_;

	/**
	 * \brief Everything in between the left and the right side
	 */
	std::vector<cv::Point2d> center_;

	/**
	 * \brief The calculated baseline for all 3 sides
	 */
	double baseLine_[3] = { 0.0, 0.0, 0.0 };

public:

	/**
	 * \brief Splits the elements based on values in X,
	 * <rigth> < rightX, <center> < leftX, the rest in <left>
	 * \param rightX The right section border in X
	 * \param leftX The left section border in X
	 */
	void split(double rightX, double leftX);

public: // getters and setter + minor functions

	/**
	 * \brief Reset the default output matrix
	 */
	void resetOutput() {
		resetOutput(frame_);
	}

	/**
	 * \brief Reset the default output matrix using custom matrix as template
	 * \param templateFrame The template to base the configuration of the output matrix on
	 */
	void resetOutput(cv::Mat& templateFrame) {
		output_ = cv::Mat::zeros(templateFrame.rows, templateFrame.cols, templateFrame.type());
	}

	/**
	 * \brief Set the class main frame reference (no pun)
	 * \param frameToSet The frame t
	 */
	void setFrame(cv::Mat& frameToSet) {
		frameToSet.copyTo(frame_); // copy ref
	}

	/**
	 * \brief Get output matrix reference
	 * \return The output matrix reference
	 */
	const cv::Mat& getOutput() const {
		return output_;
	}

	/**
	 * \brief Get the baseline (Y)
	 * \param location For which location
	 * \return The baseline (Y)
	 */
	double getBaseLine(Location location) {
		return baseLine_[locationMap.at(location)];
	}

};

inline void Line::differentiateY(std::vector<cv::Point2d>& input, std::vector<cv::Point2d>& output) {

	output.clear();

	if (input.empty())
		return;

	auto size = input.size();

	if (size == 1) {
		output.push_back(cv::Point(input.front().x, -input.front().y));
		return;
	}

	output.reserve(input.size() - 1);

	for (auto i = 1; i < size; ++i)
		output.push_back(cv::Point(input[i].x, input[i].y - input[i - 1].y));
}

inline void Line::combine(std::vector<cv::Point2d>& sourceOne, std::vector<cv::Point2d>& sourceTwo, std::vector<cv::Point2d> target, SortMethod sort) const {
	target.reserve(sourceOne.size() + sourceTwo.size());
	target.insert(target.begin(), sourceOne.begin(), sourceOne.end());
	target.insert(target.end(), sourceTwo.begin(), sourceTwo.end());
	if (sort == SortMethod::x)
		std::sort(target.begin(), target.end(), sortX);
	else if (sort == SortMethod::y)
		std::sort(target.begin(), target.end(), sortY);
}

inline unsigned char Line::getPixelIntensity(cv::Point2d& location) {
	if (frame_.empty())
		return 0;

	if (frame_.cols > location.x)
		return 0;

	if (frame_.rows > location.y)
		return 0;

	return frame_.at<uchar>(location);
}

inline void Line::generateOutput() {
	// just basic method, can be optimized.
	for (auto& e : allSparse_)
		output_.at<unsigned char>(e) = 255;
}

inline void Line::split(double rightX, double leftX) {
	if (allSparse_.empty())
		return;

	auto size = allSparse_.size();

	right_.clear();
	left_.clear();

	right_.reserve(size);
	left_.reserve(size);

	for (auto& p : allSparse_) {
		if (p.x < rightX)
			right_.push_back(p);
		else if (p.x < leftX)
			center_.push_back(p);
		else
			left_.push_back(p);
	}
}
