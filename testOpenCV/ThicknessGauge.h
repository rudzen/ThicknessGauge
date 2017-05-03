#pragma once

#include <vector>
#include "MiniCalc.h"
#include "ThicknessGaugeData.h"

#include "_cv.h"
#include "CalibrationSettings.h"
#include "Vec.h"
#include "Util.h"

#include <opencv2/opencv.hpp>
#include "TestConfig.h"
#include <vector>


using namespace _cv;

/*
The main controller class
*/
class ThicknessGauge : protected ThicknessGaugeData {

public:
	ThicknessGauge(): frameTime_(0), frameCount_(0), showWindows_(false), saveVideo_(false), binaryThreshold_(75), lineThreshold_(100) {
		baseLine_[0] = 0.0;
		baseLine_[1] = 0.0;
		baseColour_ = cv::Scalar(255, 255, 255);
		settings.ddepth = CV_8S;
	}
private:

	const double PIangle = CV_PI / 180;

	map<WindowType, string> m_WindowNames = {{WindowType::Input, "Camera Input"},{WindowType::Output, "Output"},{WindowType::Temp, "Temp"}};


public:


private:
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

public:

	double ThicknessGauge::getBaseLine(int side) const {
		return baseLine_[side];
	}

	void ThicknessGauge::setBaseLine(double baseLine, int side) {
		baseLine_[side] = baseLine;
	}
	
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

	void drawPlarnarPixels(cv::Mat& targetImage, vector<cv::Point>& planarMap) const;

	int getHighestYpixel(cv::Mat& image, int x) const;

	int getAllPixelSum(cv::Mat& image);

	static uchar getElementIntensity(cv::Mat& image, cv::Point& point);

	uchar getElementIntensity(cv::Mat& image, v2<int>& point) const;

	static double getYPixelsAvg(cv::Mat& image, int x);

	int getAllPixelSum(cv::Mat& image, int x);

	int getHighestYPixel(cv::Mat& image, int x);

	double computerBaseLine(const cv::Mat& mat, double limit);

	bool generatePlanarImage(); // <- important!

	static void addKernelTests(vector<TestConfig>& tests, float alpha, int baseSigmaX, int x, int y);
	bool testDiff();

	bool testAggressive();

	bool savePlanarImageData(string filename, vector<cv::Point>& pixels, cv::Mat& image, int highestY, std::string timeString, std::string extraInfo) const;

	double sumColumn(cv::Mat& image, int x);

	void sumColumns(cv::Mat& image, cv::Mat& target);

	void computeAllElements(cv::Mat& image);

	void computerGaugeLine(cv::Mat& output);

	bool getSparseY(cv::Mat& image, vi& output) const;

public: // getters and setters

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

public: // draw functions

	void drawText(cv::Mat* image, const string text, TextDrawPosition position);

	/**
	 * \brief Draws a horizontal line on the parsed image
	 * \param image The image to draw the line on
	 * \param pos The position in Y where the line should be drawn
	 * \param colour The scalar colour of the line
	 */
	static void drawHorizontalLine(cv::Mat* image, uint pos, cv::Scalar colour);

	static void drawVerticalLine(cv::Mat* image, uint pos, cv::Scalar colour);

	static void drawCenterAxisLines(cv::Mat* image, cv::Scalar& colour);

	void drawHorizontalLine(cv::Mat* image, uint pos) const;

	void drawVerticalLine(cv::Mat* image, uint pos) const;

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
