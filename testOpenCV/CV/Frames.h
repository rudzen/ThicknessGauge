#pragma once
#include <opencv2/core/mat.hpp>
#include <Camera/OpenCVCap.h>
#include "namespaces/cvR.h"
#include <ostream>

/**
* \brief The "main" frame container.
* Has all the information which is required for the entire process.
* This structure is contained within the vector "frameset", and is accessed
* easily by pointing.. like..
* auto frames = frameset[frameset_index].get();
*/
class Frames {

public:

    // the captured frames
    std::vector<cv::Mat> frames_;

    // means of captured frames
    std::vector<double> means_;

    // standard deviation of frames
    std::vector<double> stddevs_;

    // string of exposure
    std::string exp_ext_;

    // size of the frame
    cv::Size frame_size_;

    // the index of the frameset
    unsigned long index_;

    // the exposure used to capture the frames
    unsigned long exp_ms_;

    Frames() = delete;

    explicit Frames(const unsigned long index);

    Frames(const std::string& expExt, unsigned long expMs);

    /**
     * \brief Clear the entire structure
     */
    void clear();

    /**
     * \brief Computes mean and stddev based on the contained frames.
     */
    void compute();

    // output stream operator, outputs the entire dataset as json
    friend std::ostream& operator<<(std::ostream& os, const Frames& obj);

};
