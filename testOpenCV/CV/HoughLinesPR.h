#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"

#include "../namespaces/tg.h"
#include "../namespaces/calc.h"
#include "LinePair.h"

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

class HoughLinesPR : public BaseR {

public:

    typedef struct LineH {
        cv::Vec4f entry_;

        line_pair<float> points_;

        std::vector<cv::Point2f> elements_;

        LineH()
            : points_(cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)) { }

        LineH(cv::Vec4f entry, line_pair<float> points)
            : entry_(entry)
              , points_(points) {
            elements_.reserve(calc::round(calc::dist_manhattan(points.p1.x, points.p2.x, points.p1.y, points.p2.y)));
        }

        friend bool operator==(const LineH& lhs, const LineH& rhs) {
            return lhs.entry_ == rhs.entry_
                && lhs.points_ == rhs.points_
                && lhs.elements_ == rhs.elements_;
        }

        friend bool operator!=(const LineH& lhs, const LineH& rhs) {
            return !(lhs == rhs);
        }

        friend std::ostream& operator<<(std::ostream& os, const LineH& obj) {
            return os
                << "entry: " << obj.entry_
                << " points(1/2): " << obj.points_.p1 << '/' << obj.points_.p2
                << " elements: " << obj.elements_;
        }
    } LineH;

    struct lineHsizeSort {
        bool operator()(const LineH& l1, const LineH& l2) const {
            return l1.elements_.size() < l2.elements_.size();
        }
    } lineHsizeSort;

    struct lineHYSort {
        bool operator()(const cv::Point2f& p1, const cv::Point2f& p2) const {
            return p1.y < p2.y;
        }
    } lineHYSort;

private:

    cv::Mat output_;

    std::vector<cv::Vec4f> lines_;

    std::vector<LineH> all_lines_;

    std::vector<LineH> right_lines_;

    std::vector<LineH> left_lines_;

public:
    const std::vector<LineH>& all_lines() const {
        return all_lines_;
    }

    const std::vector<LineH>& right_lines() const {
        return right_lines_;
    }

    const std::vector<LineH>& left_lines() const {
        return left_lines_;
    }

private:
    double center_;

    double left_y_ = 0.0;

    int rho_;

    int theta_;

    double angle_;

    int threshold_;

    int min_line_len_;

    int max_line_gab_;

    const int APER_MIN = 3;

    const int APER_MAX = 7;

    double min_theta_;

    double max_theta_;

    int i_angle_limit_;

    double angle_limit_;

public:

    HoughLinesPR(const int rho, const int theta, const int threshold, const int min_line_len, const bool show_window)
        : BaseR("HoughLinesP", show_window)
          , rho_(rho)
          , theta_(theta)
          , threshold_(threshold)
          , min_line_len_(min_line_len) {
        angle_ = calc::DEGREES * theta;
        min_theta_ = 0.0;
        max_theta_ = calc::PI;
        angle_limit_ = 0;
        max_line_gab_ = 10;
        if (show_window)
            create_window();
    }

private:
    void create_window() {
        namedWindow(window_name_, cv::WINDOW_KEEPRATIO);
        cv::createTrackbar("rho", window_name_, &rho_, 3, rhocb, this);
        cv::createTrackbar("theta", window_name_, &theta_, 180, thetacb, this);
        cv::createTrackbar("threshold", window_name_, &threshold_, 100, thresholdcb, this);
        cv::createTrackbar("min len", window_name_, &min_line_len_, 200, minLineLencb, this);
        cv::createTrackbar("max gab", window_name_, &max_line_gab_, 100, maxLineGabcb, this);
    }

    void compute_borders();

    void bresenham();

    static bool split_lines_x(std::vector<LineH>& source, std::vector<LineH>& right, std::vector<LineH>& left, double x, double* leftCenter, double* rightCenter);

    // callbacks

    static void rhocb(int value, void* userData);

    static void thetacb(int value, void* userData);

    static void thresholdcb(int value, void* userData);

    static void minLineLencb(int value, void* userData);

    static void maxLineGabcb(int value, void* userData);

    void rho(int rho) {
        this->rho_ = rho;
    }

    void theta(int theta) {
        if (theta == 0)
            theta++;
        this->theta_ = theta;
        angle_ = calc::DEGREES * theta;
    }

    void threshold(int threshold) {
        this->threshold_ = threshold;
    }

public:

    void hough_horizontal();

    void draw_line(std::vector<line_pair<float>>& linePairs, cv::Scalar colour);

    void draw_lines(std::vector<cv::Vec4f>& lines, cv::Scalar colour);

    void draw_lines(std::vector<LineH>& lines, cv::Scalar colour);

    template <typename T>
    void draw_line(cv::Point_<T>& p1, cv::Point_<T>& p2, cv::Scalar colour) {
        line(output_, p1, p2, colour, 1, CV_AA);
    }

    template <typename T>
    void draw_line(T x1, T y1, T x2, T y2, cv::Scalar colour) {
        cv::Point p1(calc::round(x1), calc::round(y1));
        cv::Point p2(calc::round(x2), calc::round(y2));
        draw_line(p1, p2, colour);
    }

    void draw_line(cv::Vec4f& line, cv::Scalar colour);

    void show() const;

    void angle_limit(double angleLimit) {
        this->angle_limit_ = angleLimit;
    }

    void original(cv::Mat& newImage) {
        original_ = newImage;
        if (show_windows_)
            cvtColor(newImage, output_, CV_GRAY2BGR);
    }

    int min_line_len() const {
        return min_line_len_;
    }

    void min_line_len(int minLineLen) {
        this->min_line_len_ = minLineLen;
    }

    int max_line_gab() const {
        return max_line_gab_;
    }

    void max_line_gab(int maxLineGab) {
        this->max_line_gab_ = maxLineGab;
    }
};

inline void HoughLinesPR::compute_borders() {}

inline void HoughLinesPR::rhocb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->theta(value);
    log_time << cv::format("%s rho : %i\n", that->window_name_, value);
}

inline void HoughLinesPR::thetacb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->theta(value);
    log_time << cv::format("%s theta : %i\n", that->window_name_, value);
}

inline void HoughLinesPR::thresholdcb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->threshold(value);
    log_time << cv::format("%s threshold : %i\n", that->window_name_, value);
}

inline void HoughLinesPR::maxLineGabcb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->max_line_gab(value);
    log_time << cv::format("%s maxLineGab : %i\n", that->window_name_, value);
}

inline void HoughLinesPR::minLineLencb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->min_line_len(value);
    log_time << cv::format("%s minLineLen : %i\n", that->window_name_, value);
}

inline void HoughLinesPR::hough_horizontal() {
    // not optimized what so ever..
    // splitting things up in smaller function would help!
    lines_.reserve(image_.cols * image_.rows);

    HoughLinesP(image_, lines_, rho_, calc::PI / 4.0, threshold_, static_cast<double>(min_line_len_), static_cast<double>(max_line_gab_));

    auto count = lines_.size();

    // set up data containers.

    all_lines_.clear();
    all_lines_.reserve(count);

    left_lines_.clear();
    left_lines_.reserve(count);

    right_lines_.clear();
    right_lines_.reserve(count);

    center_ = static_cast<double>(image_.cols) * 0.5f;

    // insert lines into data structure.
    for (auto& line : lines_)
        all_lines_.emplace_back(LineH(line, line_pair<float>(line[0], line[2], line[1], line[3])));

    bresenham();

    if (show_windows_) {
        draw_lines(left_lines_, cv::Scalar(255, 0, 255));
        draw_lines(right_lines_, cv::Scalar(0, 255, 0));
        show();
    }

}

/**
 * \brief Populates the lines information for main vector and populates left and right sides
 */
inline void HoughLinesPR::bresenham() {

    if (all_lines_.empty())
        return;

    auto size = all_lines_.size();

    right_lines_.clear();
    right_lines_.reserve(size);

    left_lines_.clear();
    left_lines_.reserve(size);

    for (auto& line : all_lines_) {
        if (line.entry_[0] < center_)
            left_lines_.emplace_back(line);
        else
            right_lines_.emplace_back(line);
    }

    auto left_size = left_lines_.size();
    auto right_size = right_lines_.size(); // not wrong

    auto onlyRight = false;

    //if (rSize == 0) {
    //	if (lSize == 0)
    //		throw NoLineDetectedException("No horizontal lines detected.");

    //	// emergency case if the only located lines are on the left side.
    //	Util::copyVector(leftLines, rightLines);
    //	leftLines.clear();
    //	Util::loge("Warning, no right side lines were detected, left lines treated as right lines.");
    //	onlyRight ^= true;
    //}
    //else
    //	onlyRight = lSize == 0;

    // build right side line points
    for (auto& right_line : right_lines_) {
        cv::LineIterator it(image_, right_line.points_.p1, right_line.points_.p2, 8);
        right_line.elements_.reserve(it.count);
        for (auto i = 0; i < it.count; i++ , ++it)
            right_line.elements_.emplace_back(it.pos());
    }

    //// sort if needed
    //if (rSize > 1)
    //	sort(rightLines.begin(), rightLines.end(), lineHsizeSort);

    //// null left side lines guard
    //if (onlyRight)
    //	return;

    // build left side line points
    for (auto& left_line : left_lines_) {
        cv::LineIterator it(image_, left_line.points_.p1, left_line.points_.p2, 8);
        left_line.elements_.reserve(it.count);
        for (auto i = 0; i < it.count; i++ , ++it)
            left_line.elements_.emplace_back(it.pos());
    }

    //// sort if needed
    //if (lSize > 1)
    //	sort(leftLines.begin(), leftLines.end(), lineHsizeSort);

}

inline bool HoughLinesPR::split_lines_x(std::vector<LineH>& source, std::vector<LineH>& right, std::vector<LineH>& left, double x, double* left_center, double* right_center) {

    *left_center = 0.0;
    *right_center = 0.0;

    for (auto& line : source) {
        //if (s[1] >= yMin && s[3] >= yMin) { // desværre, ellers bliver størrelserne og dermed pointers fucked up.
        auto centerX = (line.entry_[2] + line.entry_[0]) * 0.5f;
        if (centerX <= x) {
            left.emplace_back(line);
            *left_center += centerX;
        } else {
            right.emplace_back(line);
            *right_center += centerX;
        }
        //}
    }

    if (!left.empty())
        *left_center /= left.size();

    if (!right.empty())
        *right_center /= right.size();

    return !(right.empty() && left.empty());

}

inline void HoughLinesPR::draw_line(std::vector<line_pair<float>>& line_pairs, cv::Scalar colour) {
    if (!show_windows_)
        return;

    for (auto& r : line_pairs)
        draw_line(r.p1, r.p2, colour);
}

inline void HoughLinesPR::draw_lines(std::vector<cv::Vec4f>& lines, cv::Scalar colour) {
    if (!show_windows_)
        return;

    for (auto& line : lines)
        draw_line(line, colour);
}

inline void HoughLinesPR::draw_lines(std::vector<LineH>& lines, cv::Scalar colour) {
    if (!show_windows_)
        return;

    for (auto& line : lines)
        draw_line(line.entry_, colour);
}

inline void HoughLinesPR::draw_line(cv::Vec4f& line, cv::Scalar colour) {
    draw_line(line[0], line[1], line[2], line[3], colour);
}

inline void HoughLinesPR::show() const {
    imshow(window_name_, output_);
}
