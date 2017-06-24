#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/inpainting.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "BaseR.h"
#include "../namespaces/tg.h"
#include "../namespaces/calc.h"
#include "../namespaces/validate.h"

#include "Exceptions/NoLineDetectedException.h"
#include "Exceptions/ThrowAssert.h"

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

using namespace std;
using namespace tg;

class HoughLinesR : public BaseR {

public:

    enum class Side {
        Left, Right
    };

    typedef struct LineV {
        cv::Vec2f entry_;

        tg::line_pair<float> points;

        std::vector<cv::Point_<float>> elements;

        double slobe;

        calc::SlobeDirection slobe_direction = calc::SlobeDirection::HORIZONTAL;

        LineV(cv::Vec2f entry, tg::line_pair<float> points)
            : entry_(entry)
            , points(points) {
            elements.reserve(calc::round(calc::dist_manhattan(points.p1.x, points.p2.x, points.p1.y, points.p2.y)));
            slobe = 0.0f;
        }

        friend bool operator==(const LineV& lhs, const LineV& rhs) {
            return lhs.slobe == rhs.slobe
                    && lhs.entry_ == rhs.entry_
                    && lhs.points == rhs.points
                    && lhs.elements == rhs.elements;
        }

        friend bool operator!=(const LineV& lhs, const LineV& rhs) {
            return !(lhs == rhs);
        }

        friend std::ostream& operator<<(std::ostream& os, const LineV& obj) {
            return os
                    << "entry: " << obj.entry_
                    << "slobe: " << obj.slobe
                    << " points(1/2): " << obj.points.p1 << '/' << obj.points.p2
                    << " elements: " << obj.elements;
        }
    } LineV;

private:

    // the output for visual display ONLY!
    cv::Mat output_;

    // the line output for the houghlines algorithm
    vector<cv::Vec2f> lines_;

    // the lines with all information
    vector<LineV> all_lines_;

    // the lines located on the right side of the image
    vector<LineV> right_lines_;

    // the lines location on the left side of the image
    vector<LineV> left_lines_;

    // the x coordinates of the left "border" of the marking
    cv::Vec4d left_border_;

    // the x coordinates of the right "border" of the marking
    cv::Vec4d right_border_;

    double left_y_ = 0.0;

    /* only used for UI display through open cv*/

    int rho_;

    int theta_;

    double angle_;

    int threshold_;

    const int APERTURE_MIN = 3;

    const int APERTURE_MAX = 7;

    double srn_;

    double stn_;

    double min_theta_;

    double max_theta_;

    int i_angle_limit_;

    double angle_limit_;

public:

    HoughLinesR(const int rho, const int theta, const int threshold, const bool show_window)
        : BaseR("HoughLines", show_window)
        , rho_(rho)
        , theta_(theta)
        , threshold_(threshold) {
        angle_ = calc::DEGREES * theta;
        srn_ = 0;
        stn_ = 0;
        min_theta_ = 0.0;
        max_theta_ = calc::PI;
        angle_limit_ = 0;
        if (show_window)
            create_window();
    }

    void compute_borders();

    bool is_lines_intersecting(Side side);

private:
    void create_window() {
        namedWindow(window_name_, cv::WINDOW_KEEPRATIO);
        cv::createTrackbar("rho", window_name_, &rho_, 3, rhocb, this);
        cv::createTrackbar("theta", window_name_, &theta_, 180, thetacb, this);
        cv::createTrackbar("threshold", window_name_, &threshold_, 100, thresholdcb, this);
    }

    tg::line_pair<float> compute_point_pair(cv::Vec2f& line) const;

    void draw_lines(vector<LineV>& linePairs, cv::Scalar colour);

    void draw_line(cv::Vec4d& line);

    void show_output() const;

    void compute_meta();

    static void compute_rect_from_lines(vector<LineV>& input, cv::Rect2d& output);

    // callbacks

    static void rhocb(int value, void* userData);

    static void thetacb(int value, void* userData);

    static void thresholdcb(int value, void* userData);

    // getters & setters

    void rho(int rho) {
        this->rho_ = rho;
    }

    void theta(int theta) {
        if (theta == 0)
            theta++;
        theta_ = theta;
        angle_ = calc::DEGREES * theta;
    }

    void threshold(int threshold) {
        threshold_ = threshold;
    }

public:

    int hough_vertical();

    void angle_limit(double angleLimit) {
        this->angle_limit_ = angleLimit;
    }

    void original(cv::Mat& original) {
        original_ = original;
        if (show_windows_)
            cvtColor(original_, output_, CV_GRAY2BGR);
    }

    const vector<cv::Vec2f>& lines() const {
        return lines_;
    }

    const vector<LineV>& all_lines() const {
        return all_lines_;
    }

    void all_lines(const vector<LineV>& allLines) {
        this->all_lines_ = allLines;
    }

    const vector<LineV>& right_lines() const {
        return right_lines_;
    }

    void right_lines(const vector<LineV>& rightLines) {
        this->right_lines_ = rightLines;
    }

    const vector<LineV>& left_lines() const {
        return left_lines_;
    }

    void left_lines(const vector<LineV>& leftLines) {
        this->left_lines_ = leftLines;
    }

    const cv::Vec4d& left_border() const {
        return left_border_;
    }

    const cv::Vec4d& right_border() const {
        return right_border_;
    }

    void left_border(cv::Vec4d leftBorder) {
        left_border_ = leftBorder;
    }

    void right_border(cv::Vec4d rightBorder) {
        right_border_ = rightBorder;
    }
};

inline void HoughLinesR::compute_rect_from_lines(vector<LineV>& input, cv::Rect2d& output) {

    for (auto& line : input) {
        cv::Rect2f t = cv::boundingRect(line.elements);
        output.x += t.x;
        output.y += t.y;
        output.width += t.width;
    }

    const auto size = input.size();

    output.x /= size;
    output.y /= size;
    output.width /= size;

}

inline void HoughLinesR::compute_borders() {

    cv::Rect2d left_roi(0.0, 0.0, 0.0, static_cast<double>(image_.rows));
    cv::Rect2d right_roi(0.0, 0.0, 0.0, static_cast<double>(image_.rows));

    compute_rect_from_lines(left_lines_, left_roi);
    if (!validate::validate_rect(left_roi)) {
        //for (int i = 0; i < left_lines_.size(); i++) {
        //    cv::Rect2f t = cv::boundingRect(left_lines_[i].elements);
        //    log_time << __FUNCTION__ << " bounding rect for line : " << t << std::endl;
        //}
        log_time << __FUNCTION__ << " leftRoi : " << left_roi << std::endl;
        throw_assert(!validate::validate_rect(left_roi), "Left ROI rect failed validation!!!");
    }

    compute_rect_from_lines(right_lines_, right_roi);
    if (!validate::validate_rect(right_roi)) {
        //for (auto i = 0; i < right_lines_.size(); i++) {
        //    cv::Rect2f t = cv::boundingRect(right_lines_[i].elements);
        //    log_time << __FUNCTION__ << " bounding rect for right line : " << t << std::endl;
        //}
        log_time << __FUNCTION__ << " rightRoi : " << left_roi << std::endl;
        throw_assert(!validate::validate_rect(right_roi), "Right ROI rect failed validation!!!");
    }

    auto img_height = static_cast<double>(image_.rows);

    marking_rect_.x = left_roi.x;
    marking_rect_.y = left_roi.y;
    marking_rect_.width = right_roi.x - left_roi.x + right_roi.width;
    marking_rect_.height = img_height;
    throw_assert(validate::validate_rect(marking_rect_), "Marking rect failed validation!!!");

    left_border_[0] = left_roi.x;
    left_border_[1] = img_height;
    left_border_[2] = left_roi.x + left_roi.width;
    left_border_[3] = 0.0f;
    throw_assert((validate::valid_vec<float, 4>(left_border_)), "Left border failed validation!!!");

    right_border_[0] = right_roi.x;
    right_border_[1] = 0.0f;
    right_border_[2] = right_roi.x + right_roi.width;
    right_border_[3] = img_height;
    throw_assert((validate::valid_vec<float, 4>(right_border_)), "Right border failed validation!!!");

    if (show_windows_) {
        draw_line(left_border_);
        draw_line(right_border_);
        show_output();
    }

}

inline void HoughLinesR::rhocb(int value, void* user_data) {
    auto that = static_cast<HoughLinesR*>(user_data);
    that->theta(value);
    using namespace tg;
    log_time << cv::format("%s rho : %i\n", that->window_name_, value);
}

inline void HoughLinesR::thetacb(int value, void* user_data) {
    auto that = static_cast<HoughLinesR*>(user_data);
    that->theta(value);
    using namespace tg;
    log_time << cv::format("%s theta : %i\n", that->window_name_, value);
}

inline void HoughLinesR::thresholdcb(int value, void* user_data) {
    auto that = static_cast<HoughLinesR*>(user_data);
    that->threshold(value);
    using namespace tg;
    log_time << cv::format("%s threshold : %i\n", that->window_name_, value);
}

inline int HoughLinesR::hough_vertical() {

    if (!lines_.empty())
        lines_.clear();

    //cv::HoughLines(image, lines, rho, angle, threshold, srn, stn, minTheta, maxTheta);
    HoughLines(image_, lines_, 1.0, calc::DEGREES, threshold_, 0, 0);

    if (lines_.empty())
        return -1;

    all_lines_.clear();
    all_lines_.reserve(lines_.size());

    auto pos = 0;

    for (auto& line : lines_) {
        auto theta = line[1];
        if (theta <= calc::DEGREES * (180 - angle_limit_) && theta >= calc::DEGREES * angle_limit_)
            continue;

        //log_time << "vhough1\n";
        auto p = compute_point_pair(line);
        all_lines_.emplace_back(LineV(line, p));
        pos++;
    }

    //log_time << __FUNCTION__ << " all line count : " << all_lines_.size() << std::endl;

    if (all_lines_.empty())
        return -2;
    //cerr << "FATAL ERROR, NO VERTICAL LINES DETECTED!";

    //log_time << "vhough2\n";

    compute_meta();

    return 0;

}

inline tg::line_pair<float> HoughLinesR::compute_point_pair(cv::Vec2f& line) const {
    auto rho = line[0];
    auto theta = line[1];
    double a = cos(theta);
    double b = sin(theta);
    auto x0 = a * rho;
    auto y0 = b * rho;
    cv::Point2f pt1(static_cast<float>(x0 + 1000 * (-b)), static_cast<float>(y0 + 1000 * (a)));
    cv::Point2f pt2(static_cast<float>(x0 - 1000 * (-b)), static_cast<float>(y0 - 1000 * (a)));
    return tg::line_pair<float>(pt1, pt2);
}

inline void HoughLinesR::draw_lines(vector<LineV>& line_pairs, cv::Scalar colour) {
    if (!show_windows_)
        return;

    if (line_pairs.empty())
        return;

    for (auto& lineV : line_pairs) {
        line(output_, lineV.points.p1, lineV.points.p2, colour, 1, CV_AA);
        //line(original, r.first, r.second, colour, 1, CV_AA);
    }
}

inline void HoughLinesR::draw_line(cv::Vec4d& line) {
    cv::Point2d p1(line[0], line[1]);
    cv::Point2d p2(line[2], line[3]);
    cv::line(output_, p1, p2, cv::Scalar(120, 120, 255), 2);
}

inline void HoughLinesR::show_output() const {
    if (!show_windows_)
        return;

    imshow(window_name_, output_);
}
