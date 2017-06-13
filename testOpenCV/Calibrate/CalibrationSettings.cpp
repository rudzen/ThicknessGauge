#include "CalibrationSettings.h"

CalibrationSettings::CalibrationSettings()
    : calibration_Time(0), nrOfFrames(0), image_Width(0), image_Height(0), board_Width(0), board_Height(0), square_Size(0), FixAspectRatio(0), flagValue(0), Avg_Reprojection_Error(0) {
}

CalibrationSettings::~CalibrationSettings() {
}

int CalibrationSettings::readSettings(const string calibrationFile) {
    cv::FileStorage fs;
    fs.open(calibrationFile, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        cerr << "Failed to open " << calibrationFile << endl;
        return -1;
    }

    //calibration_Time
    fs["nrOfFrames"] >> nrOfFrames;
    fs["image_Width"] >> image_Width;
    fs["image_Height"] >> image_Height;
    fs["board_Width"] >> board_Width;
    fs["board_Height"] >> board_Height;
    fs["square_Size"] >> square_Size;
    fs["FixAspectRatio"] >> FixAspectRatio;

    fs["Camera_Matrix"] >> Camera_Matrix;
    fs["Distortion_Coefficients"] >> Distortion_Coefficients;
    fs["Avg_Reprojection_Error"] >> Avg_Reprojection_Error;
    fs["Per_View_Reprojection_Errors"] >> Per_View_Reprojection_Errors;
    fs["Extrinsic_Parameters"] >> Extrinsic_Parameters;
    fs["Image_points"] >> Image_points;

    fs.release();
    return 0;
}
