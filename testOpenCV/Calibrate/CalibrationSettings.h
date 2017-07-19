#pragma once

#include <opencv2/opencv.hpp>

using namespace std;

class CalibrationSettings {
public:

    // basic information

    int calibration_time_;
    int no_of_frames;
    int image_w_;
    int image_h_;
    int board_w_;
    int board_h_;
    int square_size_; /* mm */
    double fix_aspect_ratio_;

    // matricies etc.

    int flag_value_;

    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;

    double avg_reprojection_err_;
    cv::Mat per_view_reprojection_err_;

    cv::Mat extrinsic_params_;

    cv::Mat image_points_;

public:
    CalibrationSettings();
    ~CalibrationSettings();

    int readSettings(const string settingFileName);

};
