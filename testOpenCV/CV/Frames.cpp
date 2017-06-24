#include "Frames.h"

Frames::Frames(const unsigned long index)
    : index_(index)
    , exp_ms_(0) { }

Frames::Frames(const std::string& expExt, unsigned long expMs)
    : exp_ext_(expExt)
    , index_(0)
    , exp_ms_(expMs) { }

void Frames::clear() {
    frames_.clear();
    means_.clear();
    stddevs_.clear();
    exp_ext_.clear();
    exp_ms_ = 0;
}

void Frames::compute() {
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

std::ostream& operator<<(std::ostream& os, const Frames& obj) {

    os << "[Frame Structure]\n{\n";
    if (obj.frames_.empty()) {
        return os << "\t\"frameCount\": null,\n}";
    }
    os << cv::format("\t\"frameCount\": %i,\n", obj.frames_.size());
    os << "\t\"frameDimensions\": " << obj.frames_.front().size() << ",\n"; // , obj.frames_.front().rows);
    os << "\t\"frameType\": " << obj.frames_.front().type() << ",\n"; // , obj.frames_.front().rows);
    os << cv::format("\t\"frameExposureDefault\": %i,\n", obj.exp_ms_);
    os << cv::format("\t\"frameExposureExtension\": %s,\n", obj.exp_ext_);

    // means
    os << "\t\"means\":[ ";
    auto size = obj.means_.size();
    for (auto i = 0; i < size; i++) {
        os << cv::format("\"%f\"", obj.means_[i]);
        if (i != size - 1)
            os << ",";
    }
    os << " ],\n";
    os << "\t\"stddevs\":[ ";
    size = obj.stddevs_.size();
    for (auto i = 0; i < size; i++) {
        os << cv::format("\"%f\"", obj.stddevs_[i]);
        if (i != size - 1)
            os << ",";
    }
    return os << " ]\n}\n";
}
