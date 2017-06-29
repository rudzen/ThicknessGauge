#pragma once

#include <array>
#include <ostream>
#include "namespaces/cvR.h"
#include "CV/Frames.h"

class ThicknessGaugeData {

protected:

    std::array<unsigned long, 3> exposures_ = {4000, 10000, 20000};

    std::array<std::string, 3> exposures_short_ = {"_5k", "_20k", "_40k"};

    std::vector<std::unique_ptr<Frames>> frameset_;

    std::vector<cv::Mat> nulls_;

    cv::Size image_size_;

    void initFrames() {
        frameset_.clear();
        frameset_.reserve(3);
        for (auto i = 0; i < exposures_.size(); i++) {
            auto fra = std::make_unique<Frames>(exposures_short_[i], exposures_[i]);
            frameset_.emplace_back(std::move(fra));
        }
        frameset_.shrink_to_fit();
    }

public:

    ThicknessGaugeData() {
        initFrames();
    }

    void image_size(cv::Size size);

    void image_size(const int width, const int height);

};

inline void ThicknessGaugeData::image_size(cv::Size size) {
    image_size_ = size;
}

inline void ThicknessGaugeData::image_size(const int width, const int height) {
    image_size_.width = width;
    image_size_.height = height;
}
