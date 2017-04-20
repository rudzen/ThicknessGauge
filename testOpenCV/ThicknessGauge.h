#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "_cv.h"
#include "CalibrationSettings.h"

using namespace std;
using namespace _cv;

/*
The main controller class
*/
class ThicknessGauge {

public:
	ThicknessGauge(): frameTime_(0), frameCount_(0), showWindows_(false), saveVideo_(false), rightMean_(0), leftMean_(0) {
	}

private:

	map<WindowType, string> m_WindowNames = {{WindowType::Input, "Camera Input"},{WindowType::Output, "Output"},{WindowType::Temp, "Temp"}};

	cv::Mat planarImage_;

	// The pixels located in the image
	vector<cv::Point2i> pixels_;

	vector<cv::Point2d> allPixels_;
	vector<cv::Point2d> measureLine_;
	vector<cv::Point2d> rightSideLine_;
	vector<cv::Point2d> leftSideLine_;

public:
	const vector<cv::Point2i>& getPixels() const;
	const vector<cv::Point2d>& getAllPixels() const;
	const vector<cv::Point2d>& getMeasureLine() const;
	const vector<cv::Point2d>& getRightSideLine() const;
	const vector<cv::Point2d>& getLeftSideLine() const;
private:
	uint64 frameTime_;

	int frameCount_;

public:
	int getFrameCount() const;
	void setFrameCount(int frameCount);
	uint64 getFrameTime() const;
	void setFrameTime(uint64 uint64);
private:
	double const tickFrequency_ = cv::getTickFrequency();

public:
	double getTickFrequency() const;
private:
	bool showWindows_;
	bool saveVideo_;

public:
	bool isSaveVideo() const;
	void setSaveVideo(bool saveVideo);
	bool isShowWindows() const;
	void setShowWindows(bool showWindows);
	cv::Mat& GetPlanarImage();
	void setPlanarImage(const cv::Mat& mat);
private:
	double rightMean_;
	double leftMean_;

	// laplace/sobel settings
	int kernelSize_ = 3;
	int scale_ = 1;
	int delta_ = 0;
	int ddepth_ = CV_16S;

	const int pixelChunkCount_ = 16;
	const int pixelChunkSize_ = 153; // lowest whole number from 2448 as (2448 >> 4 = 153)

public: // opencv and misc settings objects

	cv::VideoCapture cap;
	CalibrationSettings cs;

	void initVideoCapture() {
		cap.open(CV_CAP_PVAPI);
	}

	void initCalibrationSettings(string fileName) {
		cs.readSettings(fileName);
	}

public: // basic stuff to extract information

	void gatherPixels(cv::Mat& image);

	void Blur(cv::Mat& image, cv::Size size) const {
		GaussianBlur(image, image, size, 1.5, 1.5);
	}

	static void MeanReduction(cv::Mat& image) {
		MeanReduction(image);
	}

	void laplace(cv::Mat& image) const;

	void sobel(cv::Mat& image) const;

	static void imageSkeleton(cv::Mat& image);

	void drawPlarnarPixels(cv::Mat& targetImage, vector<cv::Point>& planarMap) const;

	static int getHighestYpixel(cv::Mat& image);

	bool generatePlanarImage();

	bool savePlanarImageData(string filename, vector<cv::Point>& pixels, cv::Mat& image, int highestY) const;

public: // getters and setters


	void setRightMean(double mean);

	double getRightMean() const;

	void setLeftMean(double mean);

	double getLeftMean() const;

	//vector<cv::Point2i>& GetRightSideLine();

	//vector<cv::Point2i>& GetLeftSideLine();

public: // draw functions

	static void drawText(cv::Mat* image, const string text, TextDrawPosition position);

	static void drawCenterAxisLines(cv::Mat* image);

public: // generate meta stuff

	static void GenerateInputQuad(cv::Mat* image, cv::Point2f* quad);

	static void GenerateOutputQuad(cv::Mat* image, cv::Point2f* quad);

public: // misc quad temp stuff

	void FitQuad(cv::Mat* image, cv::Point2f* inputQuad, cv::Point2f* outputQuad) const;

	static void WarpImage(cv::Mat* input, cv::Mat* output);

	static void WarpMeSomeCookies(cv::Mat* image, cv::Mat* output);

	cv::Mat cornerHarris_test(cv::Mat& image, int threshold) const;
	cv::Mat erosion(cv::Mat& input, int element, int size) const;
	cv::Mat dilation(cv::Mat& input, int dilation, int size) const;

private: // generic helper methods

	static bool IsPixel(cv::Mat& image, cv::Point& pixel) {
		return image.at<uchar>(pixel) < 255;
	}


};
