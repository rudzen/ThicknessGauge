#pragma once
#include <string>
#include <opencv2/core/mat.hpp>

class Calib {

    // simple manuel calibration of the camera..

    std::vector<cv::Mat> rvecs_;

    std::vector<cv::Mat> tvecs_;

    std::vector<std::vector<cv::Point3f>> object_points_;

    std::vector<std::vector<cv::Point2f>> image_points_;

    std::vector<cv::Point3f> obj_;

    std::vector<cv::Point2f> corners_;

    cv::Mat intrinsic_ = cv::Mat(3, 3, CV_32FC1);

    cv::Mat image_;

    cv::Mat gray_image_;

    cv::Mat dist_coeffs_;

    int board_count_ = 0;

    int corners_horizontal_ = 0;

    int corners_vertical_ = 0;

    static void show_begin();

    void save_data(std::string& filename) const;

public:

    void run_calib();

};
