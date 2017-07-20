#pragma once
#include <vector>
#include <opencv2/stitching/detail/warpers.hpp>
#include <iostream>
#include <ostream>

#include "BaseR.h"
#include "HoughLinesR.h"
#include "../Util/Vec.h"

#include "../namespaces/tg.h"

using namespace tg;

class LaserR : public BaseR {

    typedef struct xLine {

        // intensity vector
        std::vector<unsigned char> i;

        // the x pos
        int x;

        // the calculated avg y pos
        float y;

        // where x is zero
        float cut;

        xLine() {
            x = 0;
            y = 0.0f;
            cut = 0.0f;
        }

        explicit xLine(int x)
            : x(x) {
            y = 0.0f;
            cut = 0.0f;
            i.reserve(x);
        }

        friend bool operator==(const xLine& lhs, const xLine& rhs) {
            return lhs.y == rhs.y;
        }

        friend bool operator!=(const xLine& lhs, const xLine& rhs) {
            return !(lhs == rhs);
        }

        friend bool operator<(const xLine& lhs, const xLine& rhs) {
            return lhs.y < rhs.y;
        }

        friend bool operator<=(const xLine& lhs, const xLine& rhs) {
            return !(rhs < lhs);
        }

        friend bool operator>(const xLine& lhs, const xLine& rhs) {
            return rhs < lhs;
        }

        friend bool operator>=(const xLine& lhs, const xLine& rhs) {
            return !(lhs < rhs);
        }

        friend std::ostream& operator<<(std::ostream& os, const xLine& obj) {
            os << "x: " << obj.x;
            os << " y: " << obj.y;
            os << " cut:" << obj.cut;
            os << "\nv: (" << obj.i.size() << ") {\n";
            for (auto i = 0; i < obj.i.size(); i++)
                os << i << ' ' << obj.i[i] << '\n';
            os << '}';
            return os;
        }

        const cv::Point2f& xy() const {
            return cv::Point2f(static_cast<float>(x), y);
        }

    } xLine;

private:

    vector<xLine> lines_;

    bool compute_x_line();

    void configure_x_line(vector<cv::Point2i>& non_zeroes, vector<v3<float>>& output);

    // TODO : do?
    static void compute_cut(xLine& diagonal, HoughLinesR::LineV& horizontal);

public:

    bool do_laser();

    bool compute_intensity_weigth(vector<v3<float>>& output);

};

inline bool LaserR::compute_x_line() {

    auto ok = false;

    for (auto& xl : lines_) {
        if (xl.i.empty()) {
            continue;
        }

        auto sum = 0.0f;
        for (auto& intensity : xl.i) {
            sum += xl.i.size() * intensity;
        }
        xl.y = 100.0f / sum;
        ok = true;
        log_time << "x / y: " << xl.x << " / " << xl.y << endl;
    }

    return ok;

}

inline void LaserR::configure_x_line(vector<cv::Point2i>& non_zeroes, vector<v3<float>>& output) {

    if (!lines_.empty()) {
        lines_.clear();
    }

    lines_.reserve(image_.cols);

    // populate xLine vector
    for (auto i = 0; i < image_.cols; i++) {
        lines_.emplace_back(xLine(i));
    }

    // copy the values from nonZero vector
    for (auto& nz : non_zeroes) {
        lines_[nz.x].i.emplace_back(image_.at<unsigned char>(nz));
    }

    auto ok = compute_x_line();

    if (!ok) {
        log_time << "Error while calculating weighted intensity distribution.\n";
        return;
    }

    for (auto& line : lines_) {
        output.emplace_back(v3<float>(static_cast<float>(line.x), line.y, static_cast<float>(line.i.size())));
    }

}

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
