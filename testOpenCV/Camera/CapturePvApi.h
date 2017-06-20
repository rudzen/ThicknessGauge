#pragma once

#include "CaptureInterface.h"
#include <PvApi.h>

class CapturePvApi : public CaptureInterface {

    tg::tCamera myCamera;
    tPvCameraInfo cameraInfo;
    unsigned long frameSize;
    tPvErr Errcode;

    const int mono = 1;

    const cv::Rect_<unsigned long> default_roi = cv::Rect_<unsigned long>(0, 1006, 256, 2448);

    bool initialized;

    bool isOpen;

    unsigned int camera_count = 0;

    unsigned int retryCount_;

public:

    CapturePvApi() : initialized(false), isOpen(false), retryCount_(10) { }

    CapturePvApi(tg::tCamera myCamera, tPvCameraInfo cameraInfo, unsigned long frameSize)
        : myCamera(myCamera),
          cameraInfo(cameraInfo),
          frameSize(frameSize), initialized(true), isOpen(false), retryCount_(10) {
    }

public:


    unsigned retryCount() const;
    void retryCount(unsigned retryCount);
    std::string version() const;

    void region(cv::Rect_<unsigned long>& new_region);

    cv::Rect_<unsigned long> region();

    void retrieveAllInfo() override;

    void capture(int frame_count, std::vector<cv::Mat>& target_vector, unsigned long exposure) override;

    void capture(int frame_count, std::vector<cv::Mat>& target_vector);

    bool initialize();

    bool open();

    void close();

    void exposure(unsigned long new_value);

    unsigned long exposure();

    void exposure_add(unsigned long value_to_add);

    void exposure_sub(unsigned long value_to_sub);

    void exposure_mul(unsigned long value_to_mul);



};
