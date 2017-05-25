#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>

#include "Calc/MiniCalc.h"
#include "ThicknessGaugeData.h"

#include "tg.h"
#include "Calibrate/CalibrationSettings.h"
#include "Util/Util.h"

#include "Testing/TestConfig.h"
#include "LineSparse.h"
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
	ThicknessGauge(): frameTime_(0), frameCount_(0), showWindows_(false), saveVideo_(false), binaryThreshold_(20), lineThreshold_(100) {
		baseLine_[0] = 0.0;
		baseLine_[1] = 0.0;
		baseColour_ = cv::Scalar(255, 255, 255);
		settings.ddepth = CV_8S;
	}

private:

	const double PIangle = CV_PI / 180;

	uint64 frameTime_;

	int frameCount_;

	double const tickFrequency_ = cv::getTickFrequency();

	bool showWindows_;
	bool saveVideo_;

	typedef struct Settings {
		int kernelSize:1;
		int scale:1;
		int delta:1;
		int ddepth:1;

		Settings() : kernelSize(3), scale(1), delta(0), ddepth(CV_16S) {
		}

		Settings(int init_kernelSize, int init_scale, int init_delta, int init_ddepth) {
			kernelSize = init_kernelSize;
			scale = init_scale;
			delta = init_delta;
			ddepth = init_ddepth;
		}
	} Settings;

	Settings settings;

	int binaryThreshold_;

	int lineThreshold_;
	
	vi line_[2];

#ifdef CV_VERSION
	cv::Vec2d baseLine_;
#else
	double baseLine_[2];
#endif

	cv::Scalar baseColour_;

	GlobGenerator globGenerator;

	std::unique_ptr<DrawHelper> draw = make_unique<DrawHelper>(cv::Scalar(255, 255, 255));

public:

	// opencv and misc settings objects

	cv::VideoCapture cap;
	CalibrationSettings cs;

	MiniCalc miniCalc;

	void initVideoCapture();

	void initCalibrationSettings(string fileName);

public: // basic stuff to extract information

	void generateGlob(std::string& name);

	void computeMarkingHeight(std::string& globName);

	/**
	* \brief Loads all null images from "./null/" folder.
	 */
	void addNulls();

private:

	void computeBaseLineAreas(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, shared_ptr<MorphR> morph, cv::Vec4f& output);

	void computerMarkingRectangle(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesR> hough, cv::Rect2f& output);

	void computeLaserLocations(shared_ptr<LaserR> laser, cv::Vec4f& baseLine, shared_ptr<FilterR> filter, cv::Rect2f& markingLocation, std::vector<cv::Point2f>& result);

private: /* helper functions */

	cv::Vec2f computeIntersectionCut(shared_ptr<HoughLinesR> hough);

	template <int minLen>
	int computeHoughPMinLine(cv::Rect2f& rect) const;

	void splitFrames(vector<cv::Mat>& left, vector<cv::Mat>& right);

	void loadGlob(std::string& globName);

	void captureFrames();

	bool saveData(string filename, std::vector<cv::Point2f>& baseLineLeft, std::vector<cv::Point2f>& baseLineRight, std::vector<cv::Point2f>& mainLine);

	bool savePlanarImageData(string filename, vector<cv::Point>& pixels, cv::Mat& image, double highestY, std::string timeString, std::string extraInfo) const;

private: /* old functions, might be deleted */

	void laplace(cv::Mat& image) const;

	void sobel(cv::Mat& image) const;

	double sumColumn(cv::Mat& image, int x);

	void sumColumns(cv::Mat& image, cv::Mat& target);

	void computerGaugeLine(cv::Mat& output);

	bool getSparseY(cv::Mat& image, vi& output) const;

public: // getters and setters

	double ThicknessGauge::getBaseLine(int side) const {
		return baseLine_[side];
	}

	void ThicknessGauge::setBaseLine(double baseLine, int side) {
		baseLine_[side] = baseLine;
	}

	int getFrameCount() const;

	void setFrameCount(int frameCount);

	uint64 getFrameTime() const;

	void setFrameTime(uint64 uint64);

	double getTickFrequency() const;

	bool isSaveVideo() const;

	void setSaveVideo(bool saveVideo);

	bool isShowWindows() const;

	void setShowWindows(bool showWindows);

	int getBinaryThreshold() const;

	void setBinaryThreshold(int binaryThreshold);

};
