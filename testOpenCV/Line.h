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
		bool operator()(cv::Point2i pt1, cv::Point2i pt2) const { return pt1.y < pt2.y; }
	} sortY;

	struct elementXsort {
		bool operator()(cv::Point2i pt1, cv::Point2i pt2) const { return pt1.x < pt2.x; }
	} sortX;

public:

	enum class Location {
		baseOne, baseTwo, heigthOne, heigthTwo
	};

private:

	const map<Location, int> locationBaseMap = { { Location::baseOne , 0 }, { Location::baseTwo, 1 } };
	const map<Location, int> locationHeigthMap = { { Location::heigthOne , 0 }, { Location::heigthTwo, 1 } };

	map<int, unsigned char> intensity_;

	/**
	 * \brief Differentiates points in a vector
	 * \param input The vector to be differentiated
	 * \param output The results
	 */
	static void differentiateY(std::vector<cv::Point2d>& input, std::vector<cv::Point2d>& output);

	/**
	 * \brief Differentiates the intensity levels in X direction
	 */
	void differentiateIntensity();

	/**
	 * \brief Merges the differentiated values of height and intensity into Y
	 */
	void mergeIntensity();

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
	 * \brief Area of interest based on the 4 "focus" areas
	 */
	cv::Rect roi[4];

	/**
	 * \brief All the sparse elements
	 */
	std::vector<cv::Point2i> allSparse_;

	/**
	* \brief All the sparse elements with differentiated intensity values
	*/
	std::vector<cv::Point2i> allTotal_;

	/**
	 * \brief Left side of the elements
	 */
	std::vector<cv::Point2i> leftOne_;

	/**
	* \brief Left side #2 of the elements
	*/
	std::vector<cv::Point2i> leftTwo_;

	/**
	 * \brief Right side #1 of the elements
	 */
	std::vector<cv::Point2i> rightOne_;

	/**
	* \brief Right side #2 of the elements
	*/
	std::vector<cv::Point2i> rightTwo_;

	/**
	 * \brief The calculated baseline for all 3 sides
	 */
	double baseLine_[2] = { 0.0, 0.0 };

	double heigthLin_[2] = { 0.0, 0.0 };

public:

	/**
	 * \brief Splits the elements based on values in X,
	 * <rigth> < rightX, <center> < leftX, the rest in <left>
	 * \param rightOne The right section border in X
	 * \param rightTwo The left section border in X
	 */
	void split(double leftOne, double leftTwo, double rightOne, double rightTwo);

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
	double getLine(Location location) {
		switch (location) {
		case Location::baseOne:;
		case Location::baseTwo:
			return baseLine_[locationBaseMap.at(location)];
			break;
		case Location::heigthOne:;
		case Location::heigthTwo:;
		default:;
			return heigthLin_[locationHeigthMap.at(location)];
		}
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

inline void Line::differentiateIntensity() {

	if (frame_.empty())
		return;

	if (allSparse_.empty())
		return;

	auto size = allSparse_.size();

	intensity_.clear();

	if (size == 1) {
		auto front = allSparse_.front();
		intensity_[front.x] = frame_.at<unsigned char>(front);
	}

	for (auto i = 1; i < size; ++i)
		intensity_[allSparse_[i].x] = frame_.at<unsigned char>(allSparse_[i]) - frame_.at<unsigned char>(allSparse_[i - 1]);

}

inline void Line::mergeIntensity() {

	if (allSparse_.empty())
		return;

	if (intensity_.empty())
		return;

	allTotal_.clear();

	allTotal_.reserve(allSparse_.size());

	for (auto& p : allSparse_)
		allTotal_.push_back(cv::Point(p.x, p.y + intensity_.at(p.x)));

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

inline void Line::split(double leftOne, double leftTwo, double rightOne, double rightTwo) {
	if (allSparse_.empty())
		return;

	// TODO : Validate input.

	auto size = allSparse_.size();

	rightOne_.clear();
	leftOne_.clear();

	rightOne_.reserve(size);
	leftOne_.reserve(size);

	for (auto& p : allSparse_) {
		if (p.x < leftOne)
			leftOne_.push_back(p);
		else if (p.x < leftTwo)
			leftTwo_.push_back(p);
		else if (p.x < rightOne)
			rightOne_.push_back(p);
		else
			rightTwo_.push_back(p);
	}
}
