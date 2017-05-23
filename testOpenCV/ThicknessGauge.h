#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>

#include "Calc/MiniCalc.h"
#include "ThicknessGaugeData.h"

#include "tg.h"
#include "Calibrate/CalibrationSettings.h"
#include "Util/Vec.h"
#include "Util/Util.h"

#include "Testing/TestConfig.h"
#include "LineSparse.h"
#include "IO/GlobGenerator.h"
#include "CV/Pixel.h"
#include "CV/LineData/LineBaseData.h"
#include "CV/LineData/LineLaserData.h"
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

	map<WindowType, string> m_WindowNames = {{WindowType::Input, "Camera Input"},{WindowType::Output, "Output"},{WindowType::Temp, "Temp"}};

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

	// laplace/sobel settings
	//int kernelSize_ = 3;
	//int scale_ = 1;
	//int delta_ = 0;
	//int ddepth_ = CV_16S;

	int binaryThreshold_;

	int lineThreshold_;

	vi line_[2];

	double baseLine_[2];

	cv::Scalar baseColour_;

	Line histoLine_;

	GlobGenerator globGenerator;

	Pixelz pix;

	std::unique_ptr<DrawHelper> draw = make_unique<DrawHelper>(cv::Scalar(255, 255, 255));

public:
	
	// opencv and misc settings objects

	cv::VideoCapture cap;
	CalibrationSettings cs;

	MiniCalc miniCalc;

	void initVideoCapture();

	void initCalibrationSettings(string fileName);

public: // basic stuff to extract information

	/**
	 * \brief Loads all null images from "./null/" folder.
	 */
	void addNulls();

	void loadGlob(std::string& globName);

	void captureFrames();

	static void Blur(cv::Mat& image, cv::Size size);

	static void MeanReduction(cv::Mat& image);

	void laplace(cv::Mat& image) const;

	void sobel(cv::Mat& image) const;

	void drawPlarnarPixels(cv::Mat& targetImage, vector<cv::Point>& planarMap) const;

	void generateGlob(std::string& name);

	void splitFrames(vector<cv::Mat>& left, vector<cv::Mat>& right);
	template <int minLen> int computeHoughPMinLine(cv::Rect2f& rect) const;
	void computeMarkingHeight(std::string& globName);
	void computeBaseLineAreas(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, shared_ptr<MorphR> morph, cv::Vec4f& output);
	bool computerMarkingRectangle(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesR> hough, cv::Rect2f& output);
	void computeLaserLocations(shared_ptr<LaserR> laser, cv::Vec4f& baseLine, shared_ptr<FilterR> filter, cv::Rect2f& markingLocation, std::vector<cv::Point2f>& result);

	LineBaseData findMarkingLinePairs_(std::string& globName);

	bool savePlanarImageData(string filename, vector<cv::Point>& pixels, cv::Mat& image, double highestY, std::string timeString, std::string extraInfo) const;

	double sumColumn(cv::Mat& image, int x);

	void sumColumns(cv::Mat& image, cv::Mat& target);

	void computeAllElements(cv::Mat& image);

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

private: // generic helper methods

	static bool IsPixel(cv::Mat& image, cv::Point& pixel) {
		return image.at<uchar>(pixel) < 255;
	}

	void generateVectors(vi& pix) {

		// make center		
		center_.x = imageSize_.width >> 1;
		center_.y = imageSize_.height;

		lines_.clear();
		lines_.reserve(pix.size());

		//cout << "Center : " << center_ << endl;

		sort(pix.begin(), pix.end(), this->miniCalc.sortY);

		// populate lines
		for (auto& p : pix) {

			// TODO : Get the center Y point of each X position
			lines_.push_back(v2<int>(center_.x, p.y));
			//cout << "line : " << lines_.back() << endl;
			//cout << "Manhattan : " << Util::dist_manhattan(center_.x, p.x, center_.y, p.y) << '\n';
		}


	}


};
