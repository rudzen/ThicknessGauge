#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/ring_buffer.hpp>
#include <opencv2/highgui.hpp>
#include <ostream>

/*
   |  __
   | /__\
   | X~~|			"The eternal code god
   |-\|//-.		 watches over this mess."
/|`.|'.' \			- R.A.Kohn, 2017
|,|.\~~ /||
|:||   ';||
||||   | ||
\ \|     |`.
|\X|     | |
| .'     |||
| |   .  |||
|||   |  `.| JS
||||  |   ||
||||  |   ||
`+.__._._+*/

template <class T>
class HistoPeak {

    int hist_size_ = 256;

    struct Length {
        T pos1;

        T pos2;

        T size() const {
            return pos2 - pos1 + static_cast<T>(1);
        }
    };

    struct PeakInfo {
        T pos_;

        T left_size_;

        T right_size_;

        double value_;
    };

    static PeakInfo peakInfo(T pos, T left_size, T right_size, double value) {
        PeakInfo output;
        output.pos_ = pos;
        output.left_size_ = left_size;
        output.right_size_ = right_size;
        output.value_ = value;
        return output;
    }

    cv::Mat org_ = cv::Mat::zeros(1, 1, CV_8S);

    cv::Mat hist_image_;

    std::vector<T> peaks_;

    std::vector<T> dales_;

    std::vector<cv::Point_<T>> hist_elements_;

public:

    HistoPeak() /*:  org_(_org) */ { }

    const std::vector<cv::Point2i>& hist_elements() const {
        return hist_elements_;
    }

    const cv::Mat& hist_image() const {
        return hist_image_;
    }

    cv::Mat colour_me_dirty(int lowerBoundry, int upperBoundry, cv::Scalar col) const;

    void process_image(cv::Mat& image, bool uniform, bool accumulate);

    void draw_peaks(cv::Mat& histImage, std::vector<int>& peaks) const;

    void draw_dales(cv::Mat& histImage, std::vector<int>& dales) const;

    cv::Mat draw_histogram(cv::Mat& hist, int histH = 400, int histW = 2048, int histSize = 256, cv::Scalar color = cv::Scalar(255, 255, 255), int type = 2) const;

    static std::vector<PeakInfo> find_peaks(cv::InputArray _src, int window_size);

    static std::vector<PeakInfo> find_dales(cv::InputArray _src, int windowSize);

    //if you play with the peak_per attribute value, you can increase/decrease the number of peaks found
    std::vector<int> get_local_maximum(cv::InputArray _src, int smoothSize = 9, int neighborSize = 3, float peakPer = 0.1) const;

    std::vector<int> get_local_minimum(cv::InputArray _src, int smoothSize = 9, int neighborSize = 3, float dalePer = 0.5) const;

    friend std::ostream& operator<<(std::ostream& os, const HistoPeak& obj) {
        os << "HistoPeak {";
        os << "peaks: ";
        for (auto& p : obj.peaks_)
            os << ' ' << p;

        os << "\ndales: ";
        for (auto& d : obj.dales_)
            os << ' ' << d;
        os << "}\n";
        return os;
    }
};

template <class T>
cv::Mat HistoPeak<T>::colour_me_dirty(int lowerBoundry, int upperBoundry, cv::Scalar col) const {

    vector<cv::Point2i> pix;

    findNonZero(org_, pix);

    cv::Mat img = cv::Mat::zeros(org_.rows, org_.cols, CV_32F);

    if (pix.empty())
        return img;

    cv::cvtColor(org_, img, cv::COLOR_GRAY2BGR);

    for (const auto& p : pix) {
        auto intensity = org_.at<unsigned char>(p);
        if (intensity >= lowerBoundry && intensity <= upperBoundry)
            cv::line(img, p, p, col);
    }

    return img;
}

template <class T>
inline void HistoPeak<T>::process_image(cv::Mat& image, bool uniform, bool accumulate) {

    org_ = image.clone();

    auto minRange = 0.0;
    auto maxRange = 0.0;

    cv::minMaxLoc(image, &minRange, &maxRange);

    // default = 0, 256
    float range[] = {1, 255};

    const float* histRange = {range};

    cv::calcHist(&image, 1, nullptr, cv::Mat(), hist_image_, 1, &hist_size_, &histRange, uniform, accumulate);

    peaks_ = get_local_maximum(hist_image_);
    dales_ = get_local_minimum(hist_image_);

    if (peaks_.empty())
        return;

    T low = 0;
    T high = hist_size_;
    T split = hist_size_ / 2;

    auto size = peaks_.size();

    switch (size) {
    case 2:
        low = peaks_.front();
        high = peaks_.back();
        split = (low + high) / 2;
        break;
    case 1:
        break;
    default: ;
        //return;
    }

    auto lowest = hist_image_.rows;
    for (auto i = 1; i < split; ++i) {
        auto current = hist_image_.at<float>(i);
        if (current < lowest && current > 0) {
            lowest = calc::round(current);
        }
    }

    imshow("colour me freaky   0-30", colour_me_dirty(0, 30, cv::Scalar(0, 0, 255)));
    imshow("colour me freaky 31-100", colour_me_dirty(31, 100, cv::Scalar(0, 0, 255)));
    imshow("colour me freaky 101-150", colour_me_dirty(101, 150, cv::Scalar(255, 0, 255)));
    imshow("colour me freaky 151-200", colour_me_dirty(151, 200, cv::Scalar(255, 0, 255)));
    imshow("colour me freaky 201-256", colour_me_dirty(201, 256, cv::Scalar(0, 255, 255)));

}

template <class T>
inline void HistoPeak<T>::draw_peaks(cv::Mat& histImage, std::vector<int>& peaks) const {
    auto bin_w = calc::round(static_cast<double>(histImage.cols) / hist_size_);
    auto col = cv::Scalar(255, 0, 255);
    for (const auto& p : peaks)
        line(histImage, cv::Point(bin_w * p, histImage.rows), cv::Point(bin_w * p, 0), col, 2);

    imshow("Peaks", histImage);
}

template <class T>
inline void HistoPeak<T>::draw_dales(cv::Mat& histImage, std::vector<int>& dales) const {
    auto bin_w = calc::round(static_cast<double>(histImage.cols) / hist_size_);
    auto col = cv::Scalar(255, 255, 0);
    for (const auto& d : dales)
        line(histImage, cv::Point(bin_w * d, histImage.rows), cv::Point(bin_w * d, 0), col, 2);

    imshow("Dales", histImage);
}

template <class T>
inline cv::Mat HistoPeak<T>::draw_histogram(cv::Mat& hist, int hist_h, int hist_w, int hist_size, cv::Scalar color, int type) const {
    auto bin_w = calc::round(static_cast<double>(hist_w) / hist_size);

    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

    switch (type) {
    case 1:
        for (auto i = 0; i < histImage.cols; i++) {
            const unsigned x = i;
            const unsigned y = hist_h;

            line(histImage, cv::Point(bin_w * x, y), cv::Point(bin_w * x, y - calc::round(hist.at<float>(i))), color);
        }

        break;
    case 2:
        for (auto i = 1; i < hist_size; ++i) {
            auto pt1 = cv::Point(bin_w * (i - 1), hist_h);
            auto pt2 = cv::Point(bin_w * i, hist_h);
            auto pt3 = cv::Point(bin_w * i, hist_h - calc::round(hist.at<float>(i)));
            auto pt4 = cv::Point(bin_w * (i - 1), hist_h - calc::round(hist.at<float>(i - 1)));
            cv::Point pts[] = {pt1, pt2, pt3, pt4, pt1};

            fillConvexPoly(histImage, pts, 5, color);
        }
        break;
    default:
        for (auto i = 1; i < hist_size; ++i)
            line(histImage, cv::Point(bin_w * (i - 1), hist_h - calc::round(hist.at<float>(i - 1))),
                 cv::Point(bin_w * (i), hist_h - calc::round(hist.at<float>(i))), color, 1, 8, 0);

        break;
    }

    //cv::flip(histImage, histImage, 1);

    cv::imshow("Histogram", histImage);

    return histImage;
}

template <class T>
std::vector<typename HistoPeak<T>::PeakInfo> HistoPeak<T>::find_peaks(cv::InputArray _src, int window_size) {
    auto src = _src.getMat();

    auto slope_mat = src.clone();

    // Transform initial matrix into 1channel, and 1 row matrix
    auto src2 = src.reshape(1, 1);

    auto size = window_size / 2;

    Length up_hill, down_hill;
    std::vector<PeakInfo> output;

    auto pre_state = 0;
    auto i = size;

    while (i < src2.cols - size) {
        auto cur_state = src2.at<float>(i + size) - src2.at<float>(i - size);

        if (cur_state > 0)
            cur_state = 2;
        else if (cur_state < 0)
            cur_state = 1;
        else
            cur_state = 0;

        // In case you want to check how the slope looks like
        slope_mat.at<float>(i) = cur_state;

        if (pre_state == 0 && cur_state == 2)
            up_hill.pos1 = i;
        else if (pre_state == 2 && cur_state == 1) {
            up_hill.pos2 = i - 1;
            down_hill.pos1 = i;
        }

        if ((pre_state == 1 && cur_state == 2) || (pre_state == 1 && cur_state == 0)) {
            down_hill.pos2 = i - 1;
            auto max_pos = up_hill.pos2;
            if (src2.at<float>(up_hill.pos2) < src2.at<float>(down_hill.pos1))
                max_pos = down_hill.pos1;

            auto peak_info = peakInfo(max_pos, up_hill.size(), down_hill.size(), src2.at<float>(max_pos));

            output.emplace_back(peak_info);
        }
        i++;
        pre_state = static_cast<int>(cur_state);
    }
    return output;
}

template <class T>
std::vector<typename HistoPeak<T>::PeakInfo> HistoPeak<T>::find_dales(cv::InputArray src_, int window_size) {
    auto src = src_.getMat();

    auto slope_mat = src.clone();

    // Transform initial matrix into 1channel, and 1 row matrix
    auto src2 = src.reshape(1, 1);

    auto size = window_size / 2;

    Length up_hill;
    Length down_hill;

    std::vector<PeakInfo> output;

    auto pre_state = 0;
    auto i = size;

    while (i < src2.cols - size) {
        auto cur_state = src2.at<float>(i + size) - src2.at<float>(i - size);

        if (cur_state > 0)
            cur_state = 2;
        else if (cur_state < 0)
            cur_state = 1;
        else
            cur_state = 0;

        // In case you want to check how the slope looks like
        slope_mat.at<float>(i) = cur_state;

        if (pre_state == 0 && cur_state == 2)
            down_hill.pos1 = i;
        else if (pre_state == 2 && cur_state == 1) {
            down_hill.pos2 = i - 1;
            up_hill.pos1 = i;
        }

        if (pre_state == 1 && cur_state == 2 || pre_state == 1 && cur_state == 0) {
            up_hill.pos2 = i - 1;
            auto max_pos = down_hill.pos1;
            if (src2.at<float>(down_hill.pos1) < src2.at<float>(up_hill.pos2))
                max_pos = up_hill.pos2;

            auto peak_info = peakInfo(max_pos, up_hill.size(), down_hill.size(), src2.at<float>(max_pos));

            output.emplace_back(peak_info);
        }
        i++;
        pre_state = static_cast<int>(cur_state);
    }
    return output;
}

template <class T>
std::vector<int> HistoPeak<T>::get_local_maximum(cv::InputArray _src, int smooth_size, int neighbor_size, float peak_per) const {
    auto src = _src.getMat().clone();

    std::vector<int> output;
    cv::GaussianBlur(src, src, cv::Size(smooth_size, smooth_size), 0);
    auto peaks = find_peaks(src, neighbor_size);

    double min_val;
    double max_val;

    cv::minMaxLoc(src, &min_val, &max_val);

    for (auto& p : peaks) {
        if (p.value > max_val * peak_per && p.left_size >= 2 && p.right_Size >= 2)
            output.emplace_back(p.pos); // could be pos
    }

    auto histImg = draw_histogram(src);
    draw_peaks(histImg, output);
    return output;
}

template <class T>
std::vector<int> HistoPeak<T>::get_local_minimum(cv::InputArray _src, int smooth_size, int neighbor_size, float dale_per) const {
    auto src = _src.getMat().clone();

    std::vector<int> output;
    cv::GaussianBlur(src, src, cv::Size(smooth_size, smooth_size), 0);
    auto dales = find_dales(src, neighbor_size);

    double min_val;
    double max_val;

    cv::minMaxLoc(src, &max_val, &min_val);

    for (const auto& d : dales) {
        if (d.value > max_val * dale_per && d.left_size >= 2 && d.right_Size >= 2)
            output.emplace_back(d.pos);
    }

    auto histImg = draw_histogram(src);
    draw_dales(histImg, output);
    return output;
}
