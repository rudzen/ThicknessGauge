#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/videostab/ring_buffer.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "tg.h"

namespace draw {

    constexpr int ESC = 27;

    const cv::Scalar colour = cv::Scalar(255, 255, 255);

    inline char get_key(const int delay) {
        return static_cast<char>(cv::waitKey(delay));
    }

    inline bool is_escape_pressed(const int delay) {
        return get_key(delay) == ESC;
    }

    template <typename T>
    void makeWindow(T name) {
        cv::namedWindow(name, cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED);
    }

    template <typename T>
    void removeWindow(T name) {
        cv::destroyWindow(name);
        cv::waitKey(1);
    }

    inline void removeAllWindows() {
        cv::destroyAllWindows();
    }

    template <typename T>
    void showImage(T windowName, cv::Mat& image) {
        cv::imshow(windowName, image);
    }

    template <typename T>
    void drawLine(cv::Mat& image, cv::Point_<T>& p1, cv::Point_<T>& p2, cv::Scalar colour, int thickness) {
        cv::line(image, p1, p2, colour, thickness, CV_AA);
    }

    template <typename T>
    void drawLine(cv::Mat& image, cv::Point_<T>& p1, cv::Point_<T>& p2, cv::Scalar colour) {
        drawLine(image, p1, p2, colour, 1);
    }

    template <typename T>
    void drawLine(cv::Mat& image, cv::Vec<T, 4>& p, cv::Scalar colour) {
        drawLine(image, cv::Point_<T>(p[0], p[1]), cv::Point_<T>(p[2], p[3]), colour);
    }

    template <typename T>
    void drawLine(cv::Mat& image, T x1, T y1, T x2, T y2, cv::Scalar colour) {
        drawLine(image, cv::Point_<T>(x1, y1), cv::Point_<T>(x2, y2), colour);
    }

    inline void drawHorizontalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
        line(*image, cv::Point(0, image->rows - pos), cv::Point(image->cols, image->rows - pos), colour, 1, cv::LINE_AA);
    }

    inline void drawHorizontalLine(cv::Mat* image, uint pos) {
        drawHorizontalLine(image, pos, cv::Scalar(255, 255, 255));
    }

    inline void drawVerticalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
        line(*image, cv::Point(pos, 0), cv::Point(pos, image->rows), colour);
    }

    inline void drawVerticalLine(cv::Mat* image, uint pos) {
        drawVerticalLine(image, pos, colour);
    }

    inline void drawCenterAxisLines(cv::Mat* image, cv::Scalar colour) {
        drawHorizontalLine(image, image->rows >> 1, colour);
        drawVerticalLine(image, image->cols >> 1, colour);
    }

    inline void drawCenterAxisLines(cv::Mat* image) {
        drawCenterAxisLines(image, colour);
    }

    inline void drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position, cv::Scalar colour) {
        cv::Point pos;
        switch (position) {
        case tg::TextDrawPosition::UpperLeft: pos.x = image->cols / 3;
            pos.y = image->rows >> 2;
            break;
        case tg::TextDrawPosition::UpperRight: pos.x = image->cols - image->cols / 3;
            pos.y = image->rows >> 2;
            break;
        case tg::TextDrawPosition::LowerLeft: pos.x = image->cols / 3;
            pos.y = image->rows - 3 * (image->rows >> 2);
            break;
        case tg::TextDrawPosition::LowerRight: pos.x = image->cols - image->cols / 3;
            pos.y = image->rows - (image->rows >> 2);
            break;
        default:
            // oh noes..
            break;
        }
        putText(*image, text, pos, 1, 1.0, colour, 2);
    }

    inline void drawText(cv::Mat* image, const std::string text, tg::TextDrawPosition position) {
        drawText(image, text, position, colour);
    }

    template <typename T>
    void drawRectangle(cv::Mat& image, cv::Rect_<T>& rectangle, cv::Scalar colour) {
        cv::rectangle(image, rectangle, colour, 1, CV_AA);
    }


}
