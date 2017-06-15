#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "BaseR.h"

#include "../namespaces/tg.h"
#include "../namespaces/calc.h"

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
        cv::Vec4f entry;
        tg::linePair points;
        std::vector<cv::Point2f> elements;

        LineH() {
        }

        LineH(cv::Vec4f entry, tg::linePair points)
            : entry(entry),
              points(points) {
            elements.reserve(cvRound(calc::dist_manhattan(points.first.x, points.second.x, points.first.y, points.second.y)));
        }

        friend bool operator==(const LineH& lhs, const LineH& rhs) {
            return lhs.entry == rhs.entry
                && lhs.points == rhs.points
                && lhs.elements == rhs.elements;
        }

        friend bool operator!=(const LineH& lhs, const LineH& rhs) {
            return !(lhs == rhs);
        }

        friend std::ostream& operator<<(std::ostream& os, const LineH& obj) {
            return os
                << "entry: " << obj.entry
                << " points(1/2): " << obj.points.first << '/' << obj.points.second
                << " elements: " << obj.elements;
        }
    } LineH;

    struct lineHsizeSort {
        bool operator()(LineH l1, LineH l2) const { return l1.elements.size() < l2.elements.size(); }
    } lineHsizeSort;

    struct lineHYSort {
        bool operator()(cv::Point2f p1, cv::Point2f p2) const { return p1.y < p2.y; }
    } lineHYSort;

private:

    cv::Mat output;

    std::vector<cv::Vec4f> lines;

    std::vector<LineH> allLines;
    std::vector<LineH> rightLines;
    std::vector<LineH> leftLines;


public:
    const std::vector<LineH>& getAllLines() const {
        return allLines;
    }

    const std::vector<LineH>& getRightLines() const {
        return rightLines;
    }

    const std::vector<LineH>& getLeftLines() const {
        return leftLines;
    }

private:
    double center_;

    double leftY_ = 0.0;

    int rho_;

    int theta_;

    double angle_;

    int threshold_;

    int minLineLen_;

    int maxLineGab_;

    const int APER_MIN = 3;

    const int APER_MAX = 7;

    double minTheta_;

    double maxTheta_;

    int iAngleLimit_;

    double angleLimit_;

    bool showWindow_;

public:

    HoughLinesPR(const int rho, const int theta, const int threshold, const int minLineLen, const bool showWindow)
        : rho_(rho),
          theta_(theta),
          threshold_(threshold),
          minLineLen_(minLineLen),
          showWindow_(showWindow) {
        angle_ = calc::DEGREES * theta;
        minTheta_ = 0.0;
        maxTheta_ = CV_PI;
        angleLimit_ = 0;
        windowName = "HoughLinesP";
        maxLineGab_ = 10;
        if (showWindow)
            createWindow();
    }

private:
    void createWindow() {
        namedWindow(windowName, cv::WINDOW_KEEPRATIO);
        cv::createTrackbar("rho", windowName, &rho_, 3, rhocb, this);
        cv::createTrackbar("theta", windowName, &theta_, 180, thetacb, this);
        cv::createTrackbar("threshold", windowName, &threshold_, 100, thresholdcb, this);
        cv::createTrackbar("min len", windowName, &minLineLen_, 200, minLineLencb, this);
        cv::createTrackbar("max gab", windowName, &maxLineGab_, 100, maxLineGabcb, this);
    }

    void computeBorders();

    void bresenham();

    static tg::linePair HoughLinesPR::computePointPair(cv::Vec4f& line);

    double getAngle(cv::Vec4f& vec) const;

    double getAngle(cv::Point& p1, cv::Point& p2) const;

    double getAngle(int x1, int x2, int y1, int y2) const;

    static bool splitLinesInX(std::vector<LineH>& source, std::vector<LineH>& right, std::vector<LineH>& left, double x, double* leftCenter, double* rightCenter);

    // callbacks

    static void rhocb(int value, void* userData);

    static void thetacb(int value, void* userData);

    static void thresholdcb(int value, void* userData);

    static void minLineLencb(int value, void* userData);

    static void maxLineGabcb(int value, void* userData);

    void setRho(int rho) {
        this->rho_ = rho;
    }

    void setTheta(int theta) {
        if (theta == 0)
            theta++;
        this->theta_ = theta;
        angle_ = calc::DEGREES * theta;
    }

    void setThreshold(int threshold) {
        this->threshold_ = threshold;
    }

public:

    void doHorizontalHough();

    void drawLine(std::vector<tg::linePair>& linePairs, cv::Scalar colour);

    void drawLines(std::vector<cv::Vec4f>& lines, cv::Scalar colour);

    void drawLines(std::vector<LineH>& lines, cv::Scalar colour);

    void drawLine(cv::Point2f& p1, cv::Point2f& p2, cv::Scalar colour);

    void drawLine(cv::Point& p1, cv::Point& p2, cv::Scalar colour);

    void drawLine(int x1, int y1, int x2, int y2, cv::Scalar colour);

    void drawLine(float x1, float y1, float x2, float y2, cv::Scalar colour);

    void drawLine(cv::Vec4f& line, cv::Scalar colour);

    void show() const;

    void setAngleLimit(double angleLimit) {
        this->angleLimit_ = angleLimit;
    }

    void setOriginal(cv::Mat& newImage) {
        original_ = newImage;
        if (showWindows_)
            cvtColor(original_, output, CV_GRAY2BGR);
    }

    const int& getMinLineLen() const {
        return minLineLen_;
    }

    void setMinLineLen(int minLineLen) {
        this->minLineLen_ = minLineLen;
    }

    const int& getMaxLineGab() const {
        return maxLineGab_;
    }

    void setMaxLineGab(int maxLineGab) {
        this->maxLineGab_ = maxLineGab;
    }
};

inline void HoughLinesPR::computeBorders() {


}

inline void HoughLinesPR::rhocb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->setTheta(value);
    using namespace tg;
    log_time << cv::format("%s rho : %i\n", that->windowName, value);
}

inline void HoughLinesPR::thetacb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->setTheta(value);
    using namespace tg;
    log_time << cv::format("%s theta : %i\n", that->windowName, value);
}

inline void HoughLinesPR::thresholdcb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->setThreshold(value);
    using namespace tg;
    log_time << cv::format("%s threshold : %i\n", that->windowName, value);
}

inline void HoughLinesPR::maxLineGabcb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->setMaxLineGab(value);
    using namespace tg;
    log_time << cv::format("%s maxLineGab : %i\n", that->windowName, value);
}

inline void HoughLinesPR::minLineLencb(int value, void* userData) {
    auto that = static_cast<HoughLinesPR*>(userData);
    that->setMinLineLen(value);
    using namespace tg;
    log_time << cv::format("%s minLineLen : %i\n", that->windowName, value);
}

inline void HoughLinesPR::doHorizontalHough() {
    // not optimized what so ever..
    // splitting things up in smaller function would help!
    lines.reserve(image_.cols * image_.rows);

    HoughLinesP(image_, lines, rho_, CV_PI / 4.0, threshold_, static_cast<double>(minLineLen_), static_cast<double>(maxLineGab_));

    auto count = lines.size();

    // set up data containers.

    allLines.clear();
    allLines.reserve(count);

    leftLines.clear();
    leftLines.reserve(count);

    rightLines.clear();
    rightLines.reserve(count);

    center_ = static_cast<double>(image_.cols) * 0.5f;

    // insert lines into data structure.
    for (auto& line : lines)
        allLines.emplace_back(LineH(line, computePointPair(line)));

    bresenham();

    if (showWindows_) {
        drawLines(leftLines, cv::Scalar(255, 0, 255));
        drawLines(rightLines, cv::Scalar(0, 255, 0));
        show();
    }

}

/**
 * \brief Populates the lines information for main vector and populates left and right sides
 */
inline void HoughLinesPR::bresenham() {

    if (allLines.empty())
        return;

    auto size = allLines.size();

    rightLines.clear();
    rightLines.reserve(size);

    leftLines.clear();
    leftLines.reserve(size);

    for (auto& line : allLines) {
        if (line.entry[0] < center_)
            leftLines.emplace_back(line);
        else
            rightLines.emplace_back(line);

        //if (line.entry[0] + ((line.entry[2] - line.entry[0]) / 2) < center_)
        //	leftLines.emplace_back(line);
        //else
        //	rightLines.emplace_back(line);
    }

    auto left_size = leftLines.size();
    auto right_size = rightLines.size(); // not wrong

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
    for (auto& right_line : rightLines) {
        cv::LineIterator it(image_, right_line.points.first, right_line.points.second, 8);
        right_line.elements.reserve(it.count);
        for (auto i = 0; i < it.count; i++ , ++it)
            right_line.elements.emplace_back(it.pos());
    }

    //// sort if needed
    //if (rSize > 1)
    //	sort(rightLines.begin(), rightLines.end(), lineHsizeSort);

    //// null left side lines guard
    //if (onlyRight)
    //	return;

    // build left side line points
    for (auto& left_line : leftLines) {
        cv::LineIterator it(image_, left_line.points.first, left_line.points.second, 8);
        left_line.elements.reserve(it.count);
        for (auto i = 0; i < it.count; i++ , ++it)
            left_line.elements.emplace_back(it.pos());
    }

    //// sort if needed
    //if (lSize > 1)
    //	sort(leftLines.begin(), leftLines.end(), lineHsizeSort);

}

inline tg::linePair HoughLinesPR::computePointPair(cv::Vec4f& line) {
    return tg::linePair(cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]));
}

inline double HoughLinesPR::getAngle(cv::Vec4f& vec) const {
    return atan2(vec[1] - vec[3], vec[0] - vec[2]);
}

inline double HoughLinesPR::getAngle(cv::Point& p1, cv::Point& p2) const {
    return atan2(p1.y - p2.y, p1.x - p2.x);
}

inline double HoughLinesPR::getAngle(int x1, int x2, int y1, int y2) const {
    return atan2(y1 - y2, x1 - x2);
}

inline bool HoughLinesPR::splitLinesInX(std::vector<LineH>& source, std::vector<LineH>& right, std::vector<LineH>& left, double x, double* left_center, double* right_center) {

    *left_center = 0.0;
    *right_center = 0.0;

    for (auto& line : source) {
        //if (s[1] >= yMin && s[3] >= yMin) { // desværre, ellers bliver størrelserne og dermed pointers fucked up.
        auto centerX = (line.entry[2] + line.entry[0]) * 0.5f;
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

inline void HoughLinesPR::drawLine(std::vector<tg::linePair>& line_pairs, cv::Scalar colour) {
    if (!showWindow_)
        return;

    for (auto& r : line_pairs) {
        drawLine(r.first, r.second, colour);
        //line(original, r.first, r.second, colour, 1, CV_AA);
    }
}

inline void HoughLinesPR::drawLines(std::vector<cv::Vec4f>& lines, cv::Scalar colour) {
    if (!showWindow_)
        return;

    for (auto& line : lines)
        drawLine(line, colour);
}

inline void HoughLinesPR::drawLines(std::vector<LineH>& lines, cv::Scalar colour) {
    if (!showWindow_)
        return;

    for (auto& line : lines)
        drawLine(line.entry, colour);
}

inline void HoughLinesPR::drawLine(cv::Point2f& p1, cv::Point2f& p2, cv::Scalar colour) {
    line(output, p1, p2, colour, 1, CV_AA);
}

inline void HoughLinesPR::drawLine(cv::Point& p1, cv::Point& p2, cv::Scalar colour) {
    line(output, p1, p2, colour, 1, CV_AA);
}

inline void HoughLinesPR::drawLine(cv::Vec4f& line, cv::Scalar colour) {
    drawLine(line[0], line[1], line[2], line[3], colour);
}

inline void HoughLinesPR::drawLine(int x1, int y1, int x2, int y2, cv::Scalar colour) {
    cv::Point p1(x1, y1);
    cv::Point p2(x2, y2);
    drawLine(p1, p2, colour);
}

inline void HoughLinesPR::drawLine(float x1, float y1, float x2, float y2, cv::Scalar colour) {
    cv::Point p1(calc::round(x1), calc::round(y1));
    cv::Point p2(calc::round(x2), calc::round(y2));
    drawLine(p1, p2, colour);
}

inline void HoughLinesPR::show() const {
    imshow(windowName, output);
}
