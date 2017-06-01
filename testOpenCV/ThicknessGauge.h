#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>

#include "Calc/MiniCalc.h"
#include "ThicknessGaugeData.h"

#include "tg.h"
#include "Calibrate/CalibrationSettings.h"
#include "Util/Util.h"

#include "IO/GlobGenerator.h"
#include "CV/Pixel.h"
#include "CV/CannyR.h"
#include "CV/FilterR.h"
#include "CV/HoughLinesR.h"
#include "CV/HoughLinesPR.h"
#include "CV/LaserR.h"
#include "UI/DrawHelper.h"
#include "CV/MorphR.h"

using namespace tg;

/*
The main controller class
*/
class ThicknessGauge : protected ThicknessGaugeData {

public:
	ThicknessGauge(): data(new Data()), frameTime_(0), frameCount_(0), showWindows_(false), saveVideo_(false), binaryThreshold_(100), lineThreshold_(100) {
		baseColour_ = cv::Scalar(255, 255, 255);
	}

	using Data = struct Data {

		// the points of the laser on the marking
		std::vector<cv::Point2d> centerPoints;

		// the points thwere the laser hits ground zero on the LEFT side of the marking
		std::vector<cv::Point2d> leftPoints;

		// the points thwere the laser hits ground zero on the RIGHT side of the marking
		std::vector<cv::Point2d> rightPoints;

		// the missing link to the left
		std::vector<cv::Point2d> middleLeft;

		// the missing link to the right
		std::vector<cv::Point2d> middleRight;

		// the rectangle which includes the entire marking
		cv::Rect2d markingRect;

		// the base lines where the laser hits the actual ground-zero
		cv::Vec4d baseLines;

		// the locations for where the base lines intersect with the marking border
		cv::Vec4d intersections;

		// enclusure of the laser line
		cv::Vec4d centerLine;

		// the points where the intersections are cut.
		// adjusted so potential unwanted information is not included in further calculations
		cv::Vec2d intersectionCuts;

		// the x coordinates for the pieces that are cut out.
		cv::Vec4d middlePieces;

		// the difference between the baseline(s) and the laser line on the marking in sub-pixels
		double difference;

	};

	std::unique_ptr<Data> data; // there can be only one!

public: // data return point

	Data* getData() const {
		return data.get();
	}
	
private:

	const double PIangle = CV_PI / 180;

	std::unique_ptr<DrawHelper> draw = make_unique<DrawHelper>(cv::Scalar(255, 255, 255));

	double frameTime_;

	int frameCount_;

	bool showWindows_;

	bool saveVideo_;

	int binaryThreshold_;

	int lineThreshold_;

	cv::Scalar baseColour_;

	GlobGenerator globGenerator;

public:

	// opencv and misc settings objects

	cv::VideoCapture cap;
	CalibrationSettings cs;

	std::unique_ptr<MiniCalc> miniCalc = std::make_unique<MiniCalc>();

	void initVideoCapture();

	void initCalibrationSettings(string fileName);

public: // basic stuff to extract information

	void generateGlob(std::string& name);

	void computeMarkingHeight(std::string& globName);

	/**
	* \brief Loads all null images from "./null/" folder.
	 */
	void addNulls();

	bool saveData(string filename);

private:

	void computeBaseLineAreas(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, shared_ptr<MorphR> morph);

	static void processMatForLine(cv::Mat& org, shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, shared_ptr<MorphR> morph);

	cv::Rect2d computerMarkingRectangle(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesR> hough);

	void computeLaserLocations(shared_ptr<LaserR> laser, shared_ptr<FilterR> filter);

private: /* helper functions */

	cv::Vec2d computeIntersectionCut(shared_ptr<HoughLinesR> hough);

	static double computeHoughPMinLine(double minLen, cv::Rect2d& rect);

	void splitFrames(vector<cv::Mat>& left, vector<cv::Mat>& right, unsigned int frame_index);

	void loadGlob(std::string& globName);

	void captureFrames(unsigned int frame_index);

private:

	double sumColumn(cv::Mat& image, int x);

	void computerGaugeLine(cv::Mat& output);

	bool getSparseY(cv::Mat& image, vi& output) const;

public: // getters and setters

	int getFrameCount() const;

	void setFrameCount(int frameCount);

	double getFrameTime() const;

	bool isSaveVideo() const;

	void setSaveVideo(bool saveVideo);

	bool isShowWindows() const;

	void setShowWindows(bool showWindows);

	int getBinaryThreshold() const;

	void setBinaryThreshold(int binaryThreshold);

};
