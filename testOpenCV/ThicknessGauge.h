#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>

#include "ThicknessGaugeData.h"

#include "Calibrate/CalibrationSettings.h"

#include "UI/DrawHelper.h"

#include "IO/GlobGenerator.h"
#include "CV/Pixel.h"
#include "CV/CannyR.h"
#include "CV/FilterR.h"
#include "CV/HoughLinesR.h"
#include "CV/HoughLinesPR.h"
#include "CV/LaserR.h"
#include "CV/MorphR.h"

#include "Calc/MiniCalc.h"

#include "Vimba/GC2450MCamera.h"
#include "Camera/Capture.h"

#include "namespaces/tg.h"

using namespace tg;

/*
The main controller class
*/

class ThicknessGauge : protected ThicknessGaugeData {

public:

    ThicknessGauge(int frameCount, bool showWindows, bool saveVideo, int binaryThreshold, int lineThreshold)
        : data(std::make_shared<Data<double>>()), frameCount_(frameCount),
          showWindows_(showWindows),
          saveVideo_(saveVideo),
          binaryThreshold_(binaryThreshold),
          lineThreshold_(lineThreshold) {
        baseColour_ = cv::Scalar(255, 255, 255);
        canny = std::make_unique<CannyR>(200, 250, 3, true, showWindows, false);
    }

    //ThicknessGauge(): data(new Data()), frameCount_(0), showWindows_(false), saveVideo_(false), binaryThreshold_(100), lineThreshold_(100) {
    //	baseColour_ = cv::Scalar(255, 255, 255);
    //	canny = make_shared<CannyR>(200, 250, 3, true, showWindows_, false);
    //}


    std::shared_ptr<Data<double>> data; // there can be only one!

public: // data return point
    template <typename T>
    std::shared_ptr<Data<T>> getData() const {
        return data;
    }

    //AVT::VmbAPI::CameraPtr* getCamera() const {
    //	return data->cameraPtr.get();
    //}

private:

    std::unique_ptr<MiniCalc> miniCalc = std::make_unique<MiniCalc>();

    std::unique_ptr<DrawHelper> draw = std::make_unique<DrawHelper>(cv::Scalar(255, 255, 255));

    std::unique_ptr<Capture> capture; // initialized in initVideoCapture()

    // common canny with default settings for detecting marking borders
    std::shared_ptr<CannyR> canny;

    // filter used for marking detection
    std::unique_ptr<FilterR> filter_marking = std::make_unique<FilterR>("Marking filter");

    // filter used for base line detection
    std::unique_ptr<FilterR> filter_baseline = std::make_unique<FilterR>("Baseline filter");

    const double PIangle = CV_PI / 180;

    double frameTime_ = 0.0;

    int frameCount_;

    bool showWindows_;

    bool saveVideo_;

    int binaryThreshold_;

    int lineThreshold_;

    cv::Scalar baseColour_;

    GlobGenerator globGenerator;

public:

    // the following pointers are public on purpose!
    std::unique_ptr<CalibrationSettings> cs = std::make_unique<CalibrationSettings>();

    std::shared_ptr<Pixelz> pixels = std::make_shared<Pixelz>();

    void initialize(std::string& glob_name);

    void initVideoCapture();

    void initCalibrationSettings(string fileName) const;

public: // basic stuff to extract information

    void generateGlob(std::string& name);

    void computeMarkingHeight();

    /**
    * \brief Loads all null images from "./null/" folder.
     */
    void addNulls();

    bool saveData(string filename);

private:

    void computeBaseLineAreas(shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph);

    void processMatForLine(cv::Mat& org, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) const;

    cv::Rect2d computerMarkingRectangle(shared_ptr<HoughLinesR>& hough);

    void computeLaserLocations(shared_ptr<LaserR>& laser, shared_ptr<FilterR>& filter);

    void computerInBetween(shared_ptr<FilterR>& filter, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph);

private: /* helper functions */

    static cv::Vec2d computeIntersectionCut(shared_ptr<HoughLinesR>& hough);

    static double computeHoughPMinLine(double minLen, cv::Rect2d& rect);

    void splitFrames(vector<cv::Mat>& left, vector<cv::Mat>& right, unsigned int frame_index);

    void loadGlob(std::string& globName);

    void captureFrames(unsigned int frame_index, unsigned int capture_count, unsigned long long int exposure);

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
