// testOpenCV
// Rudy Alex Kohn ruak
// 26 06 2017
// 12:54
// Based loosely on work by Adrian Kaehler and Gary Bradski.

#pragma once
#include <opencv2/core/mat.hpp>
#include <memory>
#include "CapturePvApi.h"

class Calib2 {

    std::shared_ptr<CapturePvApi> pcapture = std::make_shared<CapturePvApi>();

    float image_sf = 0.5f;

    float delay = 1.f;

    int n_boards = 0; // Will be set by input list

    int board_w = 0;

    int board_h = 0;

    cv::Size board_sz;

    static void show_begin();

    /**
     * \brief Initializes the capture unit etc
     * \return true if everything seems ok, otherwise false
     */
    bool init();

public:

    /**
     * \brief Run the calibration.
     * It requests several things as manuel input:
     * Number of squares, number of corners and how many boards to process.
     * \return 
     */
    int calib();

};
