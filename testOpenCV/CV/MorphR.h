#pragma once
#include "BaseR.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class MorphR : public BaseR {

    cv::Mat output_;

    cv::Mat structure_element_;

    cv::MorphTypes method_;

    cv::MorphShapes element_shape_;

    int iterations_;

public:

    explicit MorphR(cv::MorphTypes method, const int iterations, const bool show_window)
        : BaseR("MorphR", show_window)
        , method_(method)
        , iterations_(iterations) {
        structure_element_ = cv::Mat();
        element_shape_ = cv::MORPH_RECT;
    }

    const cv::Mat& result() const {
        return output_;
    }

    cv::MorphTypes method() const {
        return method_;
    }

    cv::MorphShapes element_shape() const {
        return element_shape_;
    }

    void element_shape(cv::MorphShapes elementShape) {
        element_shape_ = elementShape;
    }

    void method(cv::MorphTypes method) {
        method_ = method;
    }

    int iterations() const {
        return iterations_;
    }

    void iterations(int iterations) {
        iterations_ = iterations;
    }

    void generate_structure_element(int size) {
        structure_element_ = getStructuringElement(element_shape_, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
    }

    void resetStructureElement() {
        structure_element_ = cv::Mat();
    }

    void morph() {
        cv::morphologyEx(image_, output_, method_, structure_element_, cv::Point(-1, -1), iterations_);
        if (show_windows_)
            show();
    }

private:

    void show() const {
        cv::imshow(window_name_, output_);
    }

};
