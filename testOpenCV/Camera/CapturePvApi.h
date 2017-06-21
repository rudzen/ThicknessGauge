#pragma once

#include "CaptureInterface.h"
#include <PvApi.h>

class CapturePvApi {

    tg::tCamera camera_;
    tPvCameraInfo camera_info_;
    unsigned long frameSize;
    tPvErr Errcode;

    const int mono = 1;

    const cv::Rect_<unsigned long> default_roi = cv::Rect_<unsigned long>(0, 1006, 2448, 256);

    bool initialized_;

    bool is_open_;

    unsigned int retry_count_;

    std::string error_last() const;

public:

    CapturePvApi() : initialized_(false), is_open_(false), retry_count_(10) { }

    CapturePvApi(tg::tCamera myCamera, tPvCameraInfo cameraInfo, unsigned long frameSize)
        : camera_(myCamera),
          camera_info_(cameraInfo),
          frameSize(frameSize), initialized_(true), is_open_(false), retry_count_(10) {
    }

    void reset();

    bool is_open() const;

    void is_open(bool new_value);

    bool initialized() const;

    void initialized(bool new_value);

    unsigned retry_count() const;

    void retry_count(unsigned new_value);

    std::string version() const;

    bool region(cv::Rect_<unsigned long> new_region);

    cv::Rect_<unsigned long> region();

    bool region_x(unsigned new_x);

    unsigned long region_x();

    bool region_y(unsigned new_y);

    unsigned long region_y();

    bool region_height(unsigned new_height);

    unsigned long region_height();

    bool region_width(unsigned new_width);

    unsigned long region_width();

    void cap(int frame_count, std::vector<cv::Mat>& target_vector, unsigned long exposure);

    bool initialize();

    void uninitialize();

    static unsigned long camera_count();

    bool open();

    void close();

    void exposure(unsigned long new_value);

    unsigned long exposure();

    void exposure_add(unsigned long value_to_add);

    void exposure_sub(unsigned long value_to_sub);

    void exposure_mul(unsigned long value_to_mul);



};
