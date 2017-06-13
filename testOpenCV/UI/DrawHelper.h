// Rudy Alex Kohn

#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include <opencv2/videostab/ring_buffer.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../namespaces/tg.h"

class DrawHelper {

private:

    cv::Scalar colour_;

    bool showWindows_;

public:

    explicit DrawHelper(cv::Scalar colour)
        : colour_(colour), showWindows_(false) {
        cv::startWindowThread();
    }

    void makeWindow(std::string name);

    void removeWindow(std::string& name) const;
    void removeWindow(const std::string& name) const;

    void removeAllWindows() const;

    void showImage(std::string& windowName, cv::Mat& image) const;
    void showImage(const std::string& name, cv::Mat& image) const;

    static char getKey(const int delay) {
        return static_cast<char>(cv::waitKey(delay));
    }

    bool isEscapePressed(const int delay) const {
        return showWindows_ && getKey(delay) == 27;
    }

    void drawLine(cv::Mat& image, cv::Point2f& p1, cv::Point2f& p2, cv::Scalar colour) const {
        drawLine(image, p1, p2, colour, 1);
    }

    static void drawLine(cv::Mat& image, cv::Point2f& p1, cv::Point2f& p2, cv::Scalar colour, int thickness) {
        cv::line(image, p1, p2, colour, thickness, CV_AA);
    }

    void drawHorizontalLine(cv::Mat* image, uint pos) const {
        drawHorizontalLine(image, pos, colour_);
    }

    static void drawHorizontalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
        line(*image, cv::Point(0, image->rows - pos), cv::Point(image->cols, image->rows - pos), colour, 1, cv::LINE_AA);
    }

    void drawVerticalLine(cv::Mat* image, uint pos) const {
        drawVerticalLine(image, pos, colour_);
    }

    static void drawVerticalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
        line(*image, cv::Point(pos, 0), cv::Point(pos, image->rows), colour);
    }

    void drawCenterAxisLines(cv::Mat* image) const {
        drawHorizontalLine(image, image->rows >> 1, colour_);
        drawVerticalLine(image, image->cols >> 1, colour_);
        //drawCenterAxisLines(image, colour_);
    }

    static void drawCenterAxisLines(cv::Mat* image, cv::Scalar colour) {
        drawHorizontalLine(image, image->rows >> 1, colour);
        drawVerticalLine(image, image->cols >> 1, colour);
        //line(*image, cv::Point(0, image->rows >> 1), cv::Point(image->cols, image->rows >> 1), colour);
        //line(*image, cv::Point(image->cols >> 1, 0), cv::Point(image->cols >> 1, image->rows), colour);
    }

    void drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position) const {
        drawText(image, text, position, colour_);
    }

    void drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position, cv::Scalar colour) const;

    void drawRectangle(cv::Mat& image, cv::Rect& rectangle, cv::Scalar colour) const {
        cv::rectangle(image, rectangle, colour, 1, CV_AA);
    }

    void drawRectangle(cv::Mat& image, cv::Rect2f& rectangle, cv::Scalar colour) const {
        cv::rectangle(image, rectangle, colour, 1, CV_AA);
    }

    void drawRectangle(cv::Mat& image, cv::Rect2d& rectangle, cv::Scalar colour) const {
        cv::rectangle(image, rectangle, colour, 1, CV_AA);
    }

    const cv::Scalar& colour() const {
        return colour_;
    }

    void colour(const cv::Scalar& colour) {
        colour_ = colour;
    }

    const bool& showWindows() const {
        return showWindows_;
    }

    void showWindows(bool showWindows) {
        showWindows_ = showWindows;
    }

};
