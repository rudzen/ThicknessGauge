#include "CalibrationSettings.h"

CalibrationSettings::CalibrationSettings()
    : calibration_time_(0), no_of_frames(0), image_w_(0), image_h_(0), board_w_(0), board_h_(0), square_size_(0), fix_aspect_ratio_(0), flag_value_(0), avg_reprojection_err_(0) {}

CalibrationSettings::~CalibrationSettings() {}

int CalibrationSettings::readSettings(const string calibration_file) {
    cv::FileStorage fs;
    fs.open(calibration_file, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        cerr << "Failed to open " << calibration_file << endl;
        return -1;
    }

    //calibration_Time
    fs["nrOfFrames"] >> no_of_frames;
    fs["image_Width"] >> image_w_;
    fs["image_Height"] >> image_h_;
    fs["board_Width"] >> board_w_;
    fs["board_Height"] >> board_h_;
    fs["square_Size"] >> square_size_;
    fs["FixAspectRatio"] >> fix_aspect_ratio_;

    fs["Camera_Matrix"] >> camera_matrix_;
    fs["Distortion_Coefficients"] >> distortion_coeffs_;
    fs["Avg_Reprojection_Error"] >> avg_reprojection_err_;
    fs["Per_View_Reprojection_Errors"] >> per_view_reprojection_err_;
    fs["Extrinsic_Parameters"] >> extrinsic_params_;
    fs["Image_points"] >> image_points_;

    fs.release();
    return 0;
}
