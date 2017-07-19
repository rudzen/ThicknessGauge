#pragma once
#include <map>
#include <array>
#include <ostream>
#include <opencv2/core/mat.hpp>

#include "../namespaces/calc.h"
#include "Util/Vec.h"

/*
 * Deprecated class for histogram stuff..
 * However, could still be useful for something in the future.
 */
class Histogram {

    int value_max_;

    const int hist_size_ = 256; // bin size

    std::array<float, 256> histogram_;

    cv::Mat histogram_image_;

    cv::Mat hist_;

    cv::Mat hist_normalized_;

    int hist_w_ = 1920;

    int hist_h_ = 1080;

    int bin_w_ = calc::round(static_cast<double>(hist_w_) / hist_size_);

    std::vector<v2<int>> dale_;

private:

    void normalize_histogram_image(const float max);

public:

    Histogram()
        : value_max_(0) { }

    /**
     * \brief Populate local histogram array from external map
     * \param intensity_map The intensity map to populate from
     */
    void populate_histogram(std::map<int, unsigned char>& intensity_map, bool create_image);

    void populate_histogram(cv::Mat& image);

    void save_simple_data(std::string& filename);

    /**
     * \brief Reset all histogram data
     */
    void nullify();

    void create_histogram_image();

    const cv::Mat& histogram_image() const {
        return histogram_image_;
    }

    void clear_histogram_image();

    friend std::ostream& operator<<(std::ostream& os, const Histogram& obj) {
        const auto space = ' ';
        for (auto i = 0; i < obj.hist_size_; i += 4) {
            os << obj.histogram_[i] << space;
            os << obj.histogram_[i + 1] << space;
            os << obj.histogram_[i + 2] << space;
            os << obj.histogram_[i + 3] << space;
        }
        return os;
    }

};
