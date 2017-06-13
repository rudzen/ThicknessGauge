#pragma once

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

public:

    virtual void retrieveAllInfo() = 0;

};
