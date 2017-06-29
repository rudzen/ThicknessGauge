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

#include "namespaces/tg.h"
#include "Camera/CapturePvApi.h"
#include "CV/Data.h"
#include "namespaces/draw.h"

using namespace tg;

/*
The main controller class
*/

class ThicknessGauge : protected ThicknessGaugeData {

public:

    ThicknessGauge(int frameCount, bool showWindows, bool saveVideo, int binaryThreshold, int lineThreshold)
        : pdata(std::make_shared<Data<double>>())
        , frame_count_(frameCount)
        , show_windows_(showWindows)
        , save_video_(saveVideo)
        , binary_threshold_(binaryThreshold)
        , line_threshold_(lineThreshold) {
        base_colour_ = cv::Scalar(255, 255, 255);
        pcanny = std::make_shared<CannyR>(130, 200, 3, true, showWindows, false);
        //draw::showWindows = showWindows;
    }

    std::shared_ptr<Data<double>> pdata;

    //AVT::VmbAPI::CameraPtr* getCamera() const {
    //	return data->cameraPtr.get();
    //}

private:

    std::unique_ptr<CapturePvApi> pcapture; // initialized in initVideoCapture()

    // common canny with default settings for detecting marking borders
    std::shared_ptr<CannyR> pcanny;

    // filter used for marking detection
    std::unique_ptr<FilterR> pfilter_marking = std::make_unique<FilterR>("Marking filter");

    // filter used for base line detection
    std::unique_ptr<FilterR> pfilter_baseline = std::make_unique<FilterR>("Baseline filter");

    double frame_time_ = 0.0;

    int frame_count_;

    bool show_windows_;

    bool save_video_;

    int binary_threshold_;

    int line_threshold_;

    cv::Scalar base_colour_;

public:

    // the following pointers are public on purpose!
    std::unique_ptr<CalibrationSettings> cs = std::make_unique<CalibrationSettings>();

    bool initialize(std::string& glob_name);

    void init_video_capture();

    void init_calibration_settings(string fileName) const;

public: // basic stuff to extract information

    void glob_generate(std::string& name);

    void compute_marking_height();

    /**
    * \brief Loads all null images from "./null/" folder.
     */
    void glob_add_nulls();

    bool save_data(string filename);

private:

    void test_edge();

    void compute_base_line_areas(shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph);

    void process_mat_for_line(cv::Mat& org, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) const;

    cv::Rect2d compute_marking_rectangle(shared_ptr<HoughLinesR>& hough);

    void compute_laser_locations(shared_ptr<LaserR>& laser, shared_ptr<FilterR>& filter);

    void computer_in_between(shared_ptr<FilterR>& filter, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph);

private: /* helper functions */

    void glob_load(std::string& globName);

    void capture_frames(unsigned int frame_index, unsigned int capture_count, unsigned long long int exposure);

    void computer_gauge_line(cv::Mat& output);

    bool sparse_y(cv::Mat& image, std::vector<cv::Point>& output) const;

public: // getters and setters

    int frame_count() const;

    void frame_count(int new_frame_count);

    double frame_time() const;

    bool save_video() const;

    void save_video(bool new_save_video_val);

    bool show_windows() const;

    void show_windows(bool new_show_windows);

    int binary_threshold() const;

    void binary_threshold(int binaryThreshold);

};
