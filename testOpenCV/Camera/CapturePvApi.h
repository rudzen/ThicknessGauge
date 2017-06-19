#pragma once

#include "CaptureInterface.h"
#include <PvApi.h>
#include "../namespaces/tg.h"

class CapturePvApi : public CaptureInterface {

    tg::tCamera myCamera;
    tPvCameraInfo cameraInfo;
    unsigned long frameSize;
    tPvErr Errcode;

    const int mono = 1;

    bool initialized;

    bool isOpen;

    unsigned int camera_count = 0;

public:

    CapturePvApi() : initialized(false), isOpen(false) { }

    CapturePvApi(tg::tCamera myCamera, tPvCameraInfo cameraInfo, unsigned long frameSize)
        : myCamera(myCamera),
          cameraInfo(cameraInfo),
          frameSize(frameSize), initialized(true), isOpen(false) {
    }

public:

    std::string version() const;

    std::string error();

    void region(cv::Rect_<unsigned long>& new_region);

    cv::Rect_<unsigned long> region();

    void retrieveAllInfo() override;

    void capture(int frame_count, double exposure) override;

    void capture(int frame_count, std::vector<cv::Mat>& target_vector);

    void initialize() override;

    void open();

    void close();

    void exposure(unsigned long new_value);

    unsigned long exposure();

    void exposure_add(unsigned long value_to_add);

    void exposure_sub(unsigned long value_to_sub);

    void exposure_mul(unsigned long value_to_mul);

};
