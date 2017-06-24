#include "Histogram.h"
#include "Util/Vec.h"

void Histogram::normalizeHistogramImage(const float max) {

    //auto modifier = max * hist_w;
    for (auto i = hist_size_; i--;) {
        histogram_[i] /= max * hist_w_;
    }
}

void Histogram::populate_histogram(std::map<int, unsigned char>& intensityMap, bool createImage) {
    if (intensityMap.empty())
        return;

    for (auto& i : intensityMap)
        histogram_[i.second]++;

    if (createImage)
        create_histogram_image();

}

void Histogram::populate_histogram(cv::Mat& image) {

    if (image.empty())
        return;

    nullify();

    auto min = 100;
    auto max = 200;

    auto count = max - min;

    float range[] = {static_cast<float>(min), static_cast<float>(max)};
    const float* ranges[] = {range};

    calcHist(&image, 1, nullptr, cv::Mat(), hist_, 1, &count, ranges, true, false);

    clear_histogram_image();

    normalize(hist_, hist_normalized_, 0, histogram_image_.rows, cv::NORM_MINMAX, -1, cv::Mat());

    for (auto i = 0; i < count; i++) {
        cv::line(histogram_image_, cv::Point(bin_w_ * (i - 1), hist_h_ - calc::round(hist_normalized_.at<float>(i - 1))),
                 cv::Point(bin_w_ * (i), hist_h_ - calc::round(hist_normalized_.at<float>(i))),
                 cv::Scalar(255, 0, 0), 1, cv::LINE_8, 0);
    }

    for (auto i = 0; i < count; ++i) {
        auto val = calc::round(hist_.at<float>(1, i));
        //if (val > 0)
        dale_.emplace_back(v2<int>(i, val));
    }

    // attempt to seperate highest peak with line
    //auto currentValley = 0;
    //for (auto i = 2; i < hist_.rows - 2; ++i) {
    //	auto val = calc::round(hist_.at<float>(1, i));
    //	if (val < 500)
    //		continue;
    //	auto previous = calc::round(hist_.at<float>(1, i - 2));
    //	if (previous < 500)
    //		continue;
    //	auto next = calc::round(hist_.at<float>(1, i + 2));
    //	if (next < 500)
    //		continue;
    //	if (next > val && previous < val) {
    //		cv::line(histogramImage_, cv::Point(i, 0), cv::Point(i, hist_h), cv::Scalar(255, 255, 255));
    //	}
    //}

}

void Histogram::save_simple_data(std::string& filename) {
    std::ofstream file(filename.c_str());

    for (auto& h : dale_) {
        file << h.x << ' ' << h.y << '\n';
    }
    file.close();
}

void Histogram::nullify() {
    for (auto i = hist_size_; i--;)
        histogram_[i] = 0;

    clear_histogram_image();
}

void Histogram::create_histogram_image() {

    // get max value
    auto max = histogram_[0];
    for (auto& i : histogram_)
        if (max < i)
            max = i;

    normalizeHistogramImage(max);

    const cv::Scalar black(0, 0, 0);

    for (auto i = 0; i < hist_size_; i++) {
        auto x = bin_w_ * i;
        line(histogram_image_, cv::Point(x, hist_h_), cv::Point(x, hist_h_ - calc::round(histogram_[i])), black, 1, cv::LINE_8, 0);
    }
}

void Histogram::clear_histogram_image() {
    histogram_image_ = cv::Mat(hist_h_, hist_w_, CV_8UC1, cv::Scalar(0, 0, 0));
    //histogramImage_ = cv::Mat::zeros(histogramImage.rows, histogramImage.cols, CV_8UC1);
}
