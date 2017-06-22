#pragma once

#include <array>
#include <ostream>
#include "namespaces/cvR.h"

class ThicknessGaugeData {

protected:

    /**
     * \brief The "main" frame container.
     * Has all the information which is required for the entire process.
     * This structure is contained within the vector "frameset", and is accessed
     * easily by pointing.. like..
     * auto frames = frameset[frameset_index].get();
     */
    template <typename T>
    struct Frames {
        std::vector<cv::Mat> frames_;

        std::vector<T> means_;

        std::vector<T> stddevs_;

        std::string exp_ext_;

        unsigned long exp_ms_;

        Frames() = delete;

        Frames(const std::string& expExt, unsigned long expMs) : exp_ext_(expExt)
                                                               , exp_ms_(expMs) { }

        /**
         * \brief Clear the entire structure
         */
        void clear() {
            frames_.clear();
            means_.clear();
            stddevs_.clear();
            exp_ext_ = "";
            exp_ms_ = 0.0;
        }

        /**
         * \brief Computes mean and stddev based on the contained frames.
         */
        void compute() {
            means_.clear();
            means_.shrink_to_fit();
            means_.reserve(frames_.size());
            stddevs_.clear();
            stddevs_.shrink_to_fit();
            stddevs_.reserve(frames_.size());

            cv::Vec2d ms;
            for (auto& frame : frames_) {
                cvr::compute_intensity_dtd_dev(frame, ms);
                means_.emplace_back(ms[0]);
                stddevs_.emplace_back(ms[1]);
            }
        }
    };

    std::array<unsigned long, 3> exposures_ = {5000, 20000, 40000};

    std::array<std::string, 3> expusures_short_ = {"_5k", "_20k", "_40k"};

    std::vector<std::unique_ptr<Frames<double>>> frameset_;

    std::vector<cv::Mat> nulls_;

    cv::Size image_size_;

    void initFrames() {
        frameset_.clear();
        frameset_.reserve(3);
        for (auto i = 0; i < exposures_.size(); i++) {
            auto fra = std::make_unique<Frames<double>>(expusures_short_[i], exposures_[i]);
            frameset_.emplace_back(std::move(fra));
        }
        frameset_.shrink_to_fit();
    }

    template <typename T>
    friend std::ostream& operator<<(std::ostream& os, const std::unique_ptr<Frames<T>> const& obj) {
        os << "[Frame Structure]\n{\n";
        if (obj->frames_.empty()) {
            return os << "\t\"frameCount\": null,\n}";
        }
        os << cv::format("\t\"frameCount\": %i,\n", obj->frames_.size());
        os << "\t\"frameDimensions\": " << obj->frames_.front().size() << ",\n"; // , obj->frames_.front().rows);
        os << "\t\"frameType\": " << obj->frames_.front().type() << ",\n"; // , obj->frames_.front().rows);
        os << cv::format("\t\"frameExposureDefault\": %i,\n", obj->exp_ms_);
        os << cv::format("\t\"frameExposureExtension\": %s,\n", obj->exp_ext_);

        // means
        os << "\t\"means\":[ ";
        auto size = obj->means_.size();
        for (auto i = 0; i < size; i++) {
            os << cv::format("\"%f\"", obj->means_[i]);
            if (i != size - 1)
                os << ",";
        }
        os << " ],\n";

        os << "\t\"stddevs\":[ ";
        size = obj->stddevs_.size();
        for (auto i = 0; i < size; i++) {
            os << cv::format("\"%f\"", obj->stddevs_[i]);
            if (i != size - 1)
                os << ",";
        }
        return os << " ]\n}\n";
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
