// testOpenCV
// Rudy Alex Kohn ruak
// 27 07 2017
// 14:55

#pragma once
#include "namespaces/draw.h"
#include "namespaces/calc.h"
#include "Camera/CapturePvApi.h"

class Autoexponering {

    std::vector<unsigned long> exp;

    const unsigned long start_expusure = 1000;

    double punkt;

public:
    CapturePvApi RealImage;

    void initiate() {
        RealImage.initialize();
        RealImage.open();
        RealImage.pixel_format();
        RealImage.reset_binning();
        RealImage.packet_size(9014);
        auto def_roi = cv::Rect_<unsigned long>(0, 0, 2448, 2050);
        //auto def_roi = cv::Rect_<unsigned long>(0, 1006, 2448, 256);
        RealImage.region(def_roi);
        RealImage.frame_init();
        RealImage.cap_init();
        RealImage.aquisition_init();

        

    }

    Autoexponering() { }

    cv::Mat get_image() { }

    int mads();


};
