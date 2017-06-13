#pragma once

#include <opencv2/opencv.hpp>

using namespace std;

class CalibrationSettings {
public:

    // basic information

    int calibration_Time;
    int nrOfFrames;
    int image_Width;
    int image_Height;
    int board_Width;
    int board_Height;
    int square_Size; /* mm */
    double FixAspectRatio;

    // matricies etc.

    int flagValue;

    cv::Mat Camera_Matrix;
    cv::Mat Distortion_Coefficients;

    double Avg_Reprojection_Error;
    cv::Mat Per_View_Reprojection_Errors;

    cv::Mat Extrinsic_Parameters;

    cv::Mat Image_points;

public:
    CalibrationSettings();
    ~CalibrationSettings();

    int readSettings(const string settingFileName);

};
