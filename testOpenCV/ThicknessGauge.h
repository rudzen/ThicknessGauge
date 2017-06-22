#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>

#include "ThicknessGaugeData.h"

#include "Calibrate/CalibrationSettings.h"

#include "IO/GlobGenerator.h"

#include "CV/CannyR.h"
#include "CV/FilterR.h"
#include "CV/HoughLinesR.h"
#include "CV/HoughLinesPR.h"
#include "CV/LaserR.h"
#include "CV/MorphR.h"

#include "Vimba/GC2450MCamera.h"

#include "namespaces/tg.h"
#include "Camera/CapturePvApi.h"
#include "namespaces/draw.h"

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
        canny = std::make_unique<CannyR>(130, 200, 3, true, showWindows, false);
        //draw::showWindows = showWindows;
    }

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

    std::unique_ptr<CapturePvApi> capture; // initialized in initVideoCapture()

    // common canny with default settings for detecting marking borders
    std::shared_ptr<CannyR> canny;

    // filter used for marking detection
    std::unique_ptr<FilterR> filter_marking = std::make_unique<FilterR>("Marking filter");

    // filter used for base line detection
    std::unique_ptr<FilterR> filter_baseline = std::make_unique<FilterR>("Baseline filter");

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

    bool initialize(std::string& glob_name);

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

    void testEdge();

    void computeBaseLineAreas(shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph);

    void processMatForLine(cv::Mat& org, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) const;

    cv::Rect2d computerMarkingRectangle(shared_ptr<HoughLinesR>& hough);

    void computeLaserLocations(shared_ptr<LaserR>& laser, shared_ptr<FilterR>& filter);

    void computerInBetween(shared_ptr<FilterR>& filter, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph);

private: /* helper functions */

    static double computeHoughPMinLine(double minLen, cv::Rect2d& rect);

    void loadGlob(std::string& globName);

    void captureFrames(unsigned int frame_index, unsigned int capture_count, unsigned long long int exposure);

private:

    void computerGaugeLine(cv::Mat& output);

    bool getSparseY(cv::Mat& image, std::vector<cv::Point>& output) const;

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
