#pragma once

#include <string>

using CaptureSettings = struct CaptureSettings {

    double brightness;

    double contrast;

    double saturation;

    double hue;

    double gain;

    double exposure;

    bool Rgb;

    double white_balance_u;

    double white_balance_v;

    bool rectification;

    double iso_speed;

    double buffersize;

};

class CaptureInterface {

protected:
    ~CaptureInterface() = default;

    bool auto_exposure = false;

    std::string status(std::string pre, bool s) {
        return pre + (s ? " ok" : " fail") + ".\n";
    }

public:

    virtual void retrieveAllInfo() = 0;

    virtual void capture(int frame_count, double exposure) = 0;

    virtual void initialize() = 0;

};
