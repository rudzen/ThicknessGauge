#include "FilterR.h"
#include "namespaces/draw.h"

void FilterR::create_window() {
    cv::namedWindow(window_name_, cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED);
    cv::createTrackbar("delta", window_name_, &deltaI_, 100, delta_cb, this);
}

void FilterR::delta_cb(int value, void* user_data) {
    auto that = static_cast<FilterR*>(user_data);
    auto oldVal = that->delta();
    that->delta(static_cast<double>(value));
    using namespace tg;
    log_time << cv::format("%s delta : %i -> %i\n", that->window_name_, oldVal, value);
}

FilterR::FilterR(std::string window_name, bool show_windows)
    : delta_(0.0)
      , ddepth_(-1)
      , border_(cv::BORDER_DEFAULT) {
    generate_kernel(3, 3, 1.0f);
    anchor_ = cv::Point(-1, -1);
    show_windows_ = show_windows;
    this->window_name_ = window_name;
    if (show_windows)
        create_window();
}

FilterR::FilterR(std::string window_name)
    : delta_(0.0)
      , ddepth_(-1)
      , border_(cv::BORDER_DEFAULT) {
    generate_kernel(3, 3, 1.0f);
    anchor_ = cv::Point(-1, -1);
    show_windows_ = false;
    this->window_name_ = window_name;
}

FilterR::FilterR(const cv::Mat& original, const cv::Mat& image, int ddepth, cv::Mat kernel, const cv::Point& anchor, double delta, int border, bool show_windows, std::string window_name)
    : BaseR(window_name, show_windows)
      , kernel_(kernel)
      , anchor_(anchor)
      , delta_(delta)
      , ddepth_(ddepth)
      , border_(border) {
    if (show_windows)
        create_window();
}

cv::Mat& FilterR::result() {
    return result_;
}

const cv::Mat& FilterR::kernel() const {
    return kernel_;
}

void FilterR::kernel(const cv::Mat& new_kernel) {
    kernel_ = new_kernel;
}

const cv::Point& FilterR::anchor() const {
    return anchor_;
}

void FilterR::anchor(const cv::Point& new_anchor) {
    anchor_ = new_anchor;
}

double FilterR::delta() const {
    return delta_;
}

void FilterR::delta(double new_delta) {
    //kernel_ += delta;
    delta_ = new_delta;
}

int FilterR::ddepth() const {
    return ddepth_;
}

void FilterR::ddepth(int new_ddepth) {
    ddepth_ = new_ddepth;
}

int FilterR::border() const {
    return border_;
}

void FilterR::border(int new_border) {
    border_ = new_border;
}

void FilterR::generate_kernel(int width, int height, float modifier) {
    kernel_ = cv::Mat::ones(width, height, CV_32F) / (static_cast<float>(width * height) * modifier);
}

void FilterR::do_filter() {
    do_filter(ddepth_, kernel_, anchor_, delta_, border_);
}

void FilterR::do_filter(int depth) {
    do_filter(depth, kernel_);
}

void FilterR::do_filter(int depth, cv::Mat& kernel) {
    do_filter(depth, kernel, anchor_);
}

void FilterR::do_filter(int depth, cv::Mat& kernel, cv::Point& anchor) {
    do_filter(depth, kernel, anchor, delta_);
}

void FilterR::do_filter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta) {
    do_filter(depth, kernel, anchor, delta, border_);
}

void FilterR::do_filter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta, int border) {
    filter2D(image_, result_, depth, kernel, anchor, delta, border);
    if (show_windows_)
        draw::showImage(window_name_, result_);
}
