#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "_cv.h"
#include "CalibrationSettings.h"
#include "Vec.h"
#include "Util.h"
#include "MiniCalc.h"

using namespace std;
using namespace _cv;

/*
The main controller class
*/
class ThicknessGauge {

public:
	ThicknessGauge(): frameTime_(0), frameCount_(0), showWindows_(false), saveVideo_(false), rightMean_(0), leftMean_(0), binaryThreshold_(35), baseLine_(40) {
	}

private:

	map<WindowType, string> m_WindowNames = {{WindowType::Input, "Camera Input"},{WindowType::Output, "Output"},{WindowType::Temp, "Temp"}};

	cv::Mat planarImage_;

	// The pixels located in the image
	vi pixels_;

	vd allPixels_;
	vd measureLine_;
	vd rightSideLine_;
	vd leftSideLine_;

	vector<p> lines_;
	p center_;

	uint64 frameTime_;

	int frameCount_;

	double const tickFrequency_ = cv::getTickFrequency();

	bool showWindows_;
	bool saveVideo_;

	cv::Size imageSize_;

	double rightMean_;
	double leftMean_;

	// laplace/sobel settings
	int kernelSize_ = 3;
	int scale_ = 1;
	int delta_ = 0;
	int ddepth_ = CV_16S;

	const int pixelChunkCount_ = 16;
	const int pixelChunkSize_ = 153; // lowest whole number from 2448 as (2448 >> 4 = 153)

	int binaryThreshold_;

	int baseLine_;

public:
	int getBaseLine() const;
	void setBaseLine(int baseLine);
	// opencv and misc settings objects

	cv::VideoCapture cap;
	CalibrationSettings cs;

	MiniCalc miniCalc;

	void initVideoCapture();

	void initCalibrationSettings(string fileName);

public: // basic stuff to extract information

	void gatherPixels(cv::Mat& image);

	static void Blur(cv::Mat& image, cv::Size size);

	static void MeanReduction(cv::Mat& image);

	void laplace(cv::Mat& image) const;

	void sobel(cv::Mat& image) const;

	static void imageSkeleton(cv::Mat& image);

	void drawPlarnarPixels(cv::Mat& targetImage, vector<cv::Point>& planarMap) const;

	static int getHighestYpixel(cv::Mat& image);

	bool generatePlanarImage();

	bool savePlanarImageData(string filename, vector<cv::Point>& pixels, cv::Mat& image, int highestY) const;

public: // getters and setters


	const vi& getPixels() const;
	const vd& getAllPixels() const;
	const vd& getMeasureLine() const;
	const vd& getRightSideLine() const;
	const vd& getLeftSideLine() const;

	int getFrameCount() const;

	void setFrameCount(int frameCount);

	uint64 getFrameTime() const;

	void setFrameTime(uint64 uint64);

	double getTickFrequency() const;

	bool isSaveVideo() const;

	void setSaveVideo(bool saveVideo);

	bool isShowWindows() const;

	void setShowWindows(bool showWindows);

	cv::Mat& GetPlanarImage();

	void setPlanarImage(const cv::Mat& mat);

	void setRightMean(double mean);

	double getRightMean() const;

	void setLeftMean(double mean);

	double getLeftMean() const;

	//vector<cv::Point2i>& GetRightSideLine();

	//vector<cv::Point2i>& GetLeftSideLine();

	int getBinaryThreshold() const;

	void setBinaryThreshold(int binaryThreshold);

public: // draw functions

	static void drawText(cv::Mat* image, const string text, TextDrawPosition position);

	static void drawBaseLine(cv::Mat* image, unsigned int pos);

	static void drawCenterAxisLines(cv::Mat* image);

public: // generate meta stuff

	/**
	* \brief Auto generate binary threshold cutoff based on pixel limit
	* \param pixelLimit The pixel limit to apply to the algorithm
	* \return The binary threshold that was generated
	*/
	int autoBinaryThreshold(unsigned int pixelLimit);



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

	void setImageSize(cv::Size size) {
		imageSize_ = size;
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
			lines_.push_back(_cv::p(center_.x, p.y));
			//cout << "line : " << lines_.back() << endl;
			//cout << "Manhattan : " << Util::dist_manhattan(center_.x, p.x, center_.y, p.y) << '\n';
		}


	}

};
