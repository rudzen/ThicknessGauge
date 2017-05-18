#pragma once
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

#include "Util/Util.h"
#include <opencv2/stitching/detail/warpers.hpp>

/**
 * \brief Line class, contains information from a singular captured frame.
 * Including defined sections for 3 sides (right/center/left), frame reference and test output matrix.
 */
class Line {
public:
	enum class Location {
		baseOne, baseTwo, heigthOne, heigthTwo
	};

private:

	enum class SortMethod {
		none, x, y
	};

	struct elementYsort {
		bool operator()(cv::Point2i pt1, cv::Point2i pt2) const { return pt1.y < pt2.y; }
	} sortY;

	struct elementXsort {
		bool operator()(cv::Point2i pt1, cv::Point2i pt2) const { return pt1.x < pt2.x; }
	} sortX;

	struct elementXsearch {
		bool operator()(cv::Point2i pt1, cv::Point2i pt2) const { return pt1.x < pt2.x; }
	} searchX;

	const map<Location, int> locationBaseMap = { { Location::baseOne , 0 }, { Location::baseTwo, 1 } };
	const map<Location, int> locationHeigthMap = { { Location::heigthOne , 0 }, { Location::heigthTwo, 1 } };

	map<int, int> intensity_;


public:
	const map<int, int>& intensity() const {
		return intensity_;
	}

	void intensity(const map<int, int>& intensity) {
		intensity_ = intensity;
	}

private:
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
	 * \brief All the complete work from the class. Diffirentiated Y values + differentiated intensity values.
	 */
	vector<cv::Point2i> allComplete_;

	/**
	* \brief All the sparse elements
	*/
	vector<cv::Point2i> allSparse_;

	/**
	* \brief All the raw pixel locations from frame
	*/
	vector<cv::Point2i> allTotal_;

	/**
	 * \brief All the sparse elements differentiated
	 */
	vector<cv::Point2i> allDifferentiated_;

	// temporary bastard
	vector<cv::Point2i> tmp;

	/**
	* \brief Left side of the elements
	*/
	vector<cv::Point2i> leftOne_;

	/**
	* \brief Left side #2 of the elements
	*/
	vector<cv::Point2i> leftTwo_;

	/**
	* \brief Right side #1 of the elements
	*/
	vector<cv::Point2i> rightOne_;

	/**
	* \brief Right side #2 of the elements
	*/
	vector<cv::Point2i> rightTwo_;

	/**
	* \brief The calculated baseline for all 3 sides
	*/
	double baseLine_[2] = { 0.0, 0.0 };

	double heigthLin_[2] = { 0.0, 0.0 };

public:

	/**
	 * \brief Differentiates points in a vector
	 * \param input The vector to be differentiated
	 * \param output The results
	 */
	static void differentiateY(vector<cv::Point2i>& input, vector<cv::Point2i>& output);


	/**
	 * \brief Diffirentiates twice :>
	 */
	void differentiateY();


	void saveAllData(string& filePrefix);

	/**
	 * \brief Differentiates the intensity levels in X direction
	 */
	void differentiateIntensity();

	bool pointSearchX(cv::Point2i i, cv::Point2i j);

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
	void combine(vector<cv::Point2d>& sourceOne, vector<cv::Point2d>& sourceTwo, vector<cv::Point2d> target, SortMethod sortX) const;

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

	bool generateSparse();

public:

	/**
	 * \brief Splits the elements based on values in X,
	 * <rigth> < rightX, <center> < leftX, the rest in <left>
	 * \param rightOne The right section border in X
	 * \param rightTwo The left section border in X
	 */
	void split(double leftOne, double leftTwo, double rightOne, double rightTwo);
	void drawPoly();

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
	void setFrame(const cv::Mat& frameToSet) {
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

inline void Line::differentiateY(vector<cv::Point2i>& input, vector<cv::Point2i>& output) {

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

inline void Line::differentiateY() {

	auto size = allSparse_.size();

	if (size == 1) {
		allDifferentiated_.push_back(cv::Point(allSparse_.front().x, -allSparse_.front().y));
		return;
	}

	tmp.reserve(allSparse_.size() - 1);

	for (auto i = 1; i < size; ++i)
		tmp.push_back(cv::Point(allSparse_[i].x, allSparse_[i].y - allSparse_[i - 1].y));

	differentiateY(tmp, allDifferentiated_);
}

inline void Line::saveAllData(string& filePrefix) {
	ofstream all(filePrefix + "_d_0_all.txt");
	ofstream sparse(filePrefix + "_d_1_sparse.txt");
	ofstream inten(filePrefix + "_d_2_intensity.txt");
	ofstream diff1(filePrefix + "_d_3_diff1.txt");
	ofstream diff2(filePrefix + "_d_4_diff2.txt");
	ofstream full(filePrefix + "_d_5_full.txt");

	for (auto& p : allTotal_)
		all << p << '\n';

	for (auto& p : allSparse_)
		sparse << p << '\n';

	for (auto& p : intensity_)
		inten << '[' << p.first << ", " << p.second << "]\n";

	for (auto& p : tmp)
		diff1 << p << '\n';

	for (auto& p : allDifferentiated_)
		diff2 << p << '\n';

	for (auto& p : allComplete_)
		full << p << '\n';

	diff2.close();
	diff1.close();
	inten.close();
	sparse.close();
	all.close();
	full.close();

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

	for (auto i = 1; i < size; ++i) {
		intensity_[allSparse_[i].x] = frame_.at<unsigned char>(allSparse_[i]) - frame_.at<unsigned char>(allSparse_[i - 1]);
		//auto val = intensity_[allSparse_[i].x];
		//cout << val << endl;
	}

}

inline void Line::mergeIntensity() {

	if (allDifferentiated_.empty())
		return;

	if (intensity_.empty())
		return;

	for (auto& incent : allDifferentiated_) {
		if (intensity_.count(incent.x))
			allComplete_.push_back(cv::Point(incent.x, intensity_[incent.x] + incent.y));
		else
			allComplete_.push_back(incent);
	}

	std::sort(allComplete_.begin(), allComplete_.end(), sortX);
}

inline void Line::combine(vector<cv::Point2d>& sourceOne, vector<cv::Point2d>& sourceTwo, vector<cv::Point2d> target, SortMethod sort) const {
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

inline bool Line::generateSparse() {

	if (!allSparse_.empty())
		allSparse_.clear();

	allSparse_.reserve(frame_.cols);

	allTotal_.clear();
	allTotal_.reserve(frame_.cols * frame_.rows);

	findNonZero(frame_, allTotal_);

	if (allTotal_.empty())
		return false;

	// sort the list in X
	std::sort(allTotal_.begin(), allTotal_.end(), sortX);

	auto y = 0;
	auto count = 0;
	auto highest = 0;

	auto x = allTotal_.front().x;

	for (auto& p : allTotal_) {
		if (p.x != x) {
			if (count > 0) {
				allSparse_.push_back(cv::Point(x, frame_.rows - y));
				count = 0;
			}
			highest = 0;
		}
		auto intensity = frame_.at<unsigned char>(p);
		if (intensity >= highest) {
			highest = p.y;
			x = p.x;
			y = p.y;
		}
		count++;
	}

	return allSparse_.empty() ^ true;

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

inline void Line::drawPoly() {
	polylines(frame_, allSparse_, false, cv::Scalar(255, 255, 255));
}
