#include <iostream>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../namespaces/tg.h"
#include "namespaces/str.h"

using namespace std;
using namespace tg;


class Settings {

public:

    Settings()
        : calibration_pattern_(), square_size_(0), num_frames_(0), aspect_ratio_(0), delay_(0), write_points_(false), write_extrinsics_(false), calib_zero_tangent_dist_(false), calib_fix_principal_point_(false), flip_vertical_(false), show_undistorsed_(false), camera_id_(0), at_image_list_(0), input_type_(), good_input_(false), flag_(0) { }

    enum class Mode { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

    enum class Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

    enum class InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

    //Write serialization for this class
    void write(cv::FileStorage& fs) const {
        fs << "{" << "BoardSize_Width" << board_size_.width
            << "BoardSize_Height" << board_size_.height
            << "Square_Size" << square_size_
            << "Calibrate_Pattern" << pattern_to_use_
            << "Calibrate_NrOfFrameToUse" << num_frames_
            << "Calibrate_FixAspectRatio" << aspect_ratio_
            << "Calibrate_AssumeZeroTangentialDistortion" << calib_zero_tangent_dist_
            << "Calibrate_FixPrincipalPointAtTheCenter" << calib_fix_principal_point_

            << "Write_DetectedFeaturePoints" << write_points_
            << "Write_extrinsicParameters" << write_extrinsics_
            << "Write_outputFileName" << output_file_name_

            << "Show_UndistortedImage" << show_undistorsed_

            << "Input_FlipAroundHorizontalAxis" << flip_vertical_
            << "Input_Delay" << delay_
            << "Input" << input_
            << "}";
    }

    //Read serialization for this class
    void read(const cv::FileNode& node) {
        node["BoardSize_Width"] >> board_size_.width;
        node["BoardSize_Height"] >> board_size_.height;
        node["Calibrate_Pattern"] >> pattern_to_use_;
        node["Square_Size"] >> square_size_;
        node["Calibrate_NrOfFrameToUse"] >> num_frames_;
        node["Calibrate_FixAspectRatio"] >> aspect_ratio_;
        node["Write_DetectedFeaturePoints"] >> write_points_;
        node["Write_extrinsicParameters"] >> write_extrinsics_;
        node["Write_outputFileName"] >> output_file_name_;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calib_zero_tangent_dist_;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calib_fix_principal_point_;
        node["Input_FlipAroundHorizontalAxis"] >> flip_vertical_;
        node["Show_UndistortedImage"] >> show_undistorsed_;
        node["Input"] >> input_;
        node["Input_Delay"] >> delay_;
        interprate();
    }

    void interprate() {
        good_input_ = true;
        if (board_size_.width <= 0 || board_size_.height <= 0) {
            cerr << "Invalid Board size: " << board_size_.width << " " << board_size_.height << endl;
            good_input_ = false;
        }
        if (square_size_ <= 10e-6) {
            cerr << "Invalid square size " << square_size_ << endl;
            good_input_ = false;
        }
        if (num_frames_ <= 0) {
            cerr << "Invalid number of frames " << num_frames_ << endl;
            good_input_ = false;
        }

        // Check for valid input
        if (input_.empty()) {
            input_type_ = InputType::INVALID;
        } else {
            if (input_[0] >= '0' && input_[0] <= '9') {
                stringstream ss(input_);
                ss >> camera_id_;
                input_type_ = InputType::CAMERA;
            } else {
                if (is_list_of_images(input_) && read_string_list(input_, image_list_)) {
                    input_type_ = InputType::IMAGE_LIST;
                    num_frames_ = (num_frames_ < static_cast<int>(image_list_.size())) ? num_frames_ : static_cast<int>(image_list_.size());
                } else {
                    input_type_ = InputType::VIDEO_FILE;
                }
            }
            if (input_type_ == InputType::CAMERA) {
                input_capture_.open(camera_id_);
            } else if (input_type_ == InputType::VIDEO_FILE) {
                input_capture_.open(input_);
            } else if (input_type_ != InputType::IMAGE_LIST && !input_capture_.isOpened()) {
                input_type_ = InputType::INVALID;
            }
        }

        if (input_type_ == InputType::INVALID) {
            cerr << " Inexistent input: " << input_;
            good_input_ = false;
        }

        flag_ = 0;
        if (calib_fix_principal_point_) {
            flag_ |= CV_CALIB_FIX_PRINCIPAL_POINT;
        }
        if (calib_zero_tangent_dist_) {
            flag_ |= CV_CALIB_ZERO_TANGENT_DIST;
        }
        if (aspect_ratio_) {
            flag_ |= CV_CALIB_FIX_ASPECT_RATIO;
        }

        calibration_pattern_ = Pattern::NOT_EXISTING;
        if (!pattern_to_use_.compare("CHESSBOARD")) {
            calibration_pattern_ = Pattern::CHESSBOARD;
        }
        if (!pattern_to_use_.compare("CIRCLES_GRID")) {
            calibration_pattern_ = Pattern::CIRCLES_GRID;
        }
        if (!pattern_to_use_.compare("ASYMMETRIC_CIRCLES_GRID")) {
            calibration_pattern_ = Pattern::ASYMMETRIC_CIRCLES_GRID;
        }

        if (calibration_pattern_ == Pattern::NOT_EXISTING) {
            log_err << cv::format("%s Inexistent camera calibration mode: %s\n", __FUNCTION__, pattern_to_use_);
            good_input_ = false;
        }

        at_image_list_ = 0;
    }

    cv::Mat next_image() {
        cv::Mat result;
        if (input_capture_.isOpened()) {
            cv::Mat view0;
            input_capture_ >> view0;
            return view0;
        }
        if (at_image_list_ < static_cast<int>(image_list_.size())) {
            result = cv::imread(image_list_[at_image_list_++], CV_LOAD_IMAGE_COLOR);
        }

        return result;
    }

    static bool read_string_list(const string& filename, vector<string>& l) {
        l.clear();
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            return false;
        }
        auto node = fs.getFirstTopLevelNode();
        if (node.type() != cv::FileNode::SEQ) {
            return false;
        }

        // TODO : Verify this for loop work as intended instead of the one below
        for (const auto& n : node) {
            l.emplace_back(static_cast<string>(n));
        }

        //auto file_node_iterator = node.begin(), it_end = node.end();
        //for (; file_node_iterator != it_end; ++file_node_iterator) {
        //    l.emplace_back(static_cast<string>(*file_node_iterator));
        //}

        return true;
    }

    static bool is_list_of_images(const string& filename) {

        return str::end_with(filename, ".xml") ||
            str::end_with(filename, ".yaml") ||
            str::end_with(filename, ".yml") ||
            str::end_with(filename, ".json");
    }

public:
    // The size of the board -> Number of items by width and height
    cv::Size board_size_; 

    // One of the Chessboard, circles, or asymmetric circle pattern
    Pattern calibration_pattern_;

     // The size of a square in your defined unit (point, millimeter,etc).
    float square_size_;

    // The number of frames to use from the input for calibration
    int num_frames_;

    // The aspect ratio
    float aspect_ratio_;

    // In case of a video input
    int delay_;

    //  Write detected feature points
    bool write_points_;

    // Write extrinsic parameters
    bool write_extrinsics_;

    // Assume zero tangential distortion
    bool calib_zero_tangent_dist_;

    // Fix the principal point at the center
    bool calib_fix_principal_point_;

    // Flip the captured images around the horizontal axis
    bool flip_vertical_;

    // The name of the file where to write
    string output_file_name_;

     // Show undistorted images after calibration
    bool show_undistorsed_;

    // The input
    string input_;

    // input camera ID
    int camera_id_;

    // input image list
    vector<string> image_list_;

    // input at image list var
    int at_image_list_;

    // input capture
    cv::VideoCapture input_capture_;

    // input type
    InputType input_type_;

    // input good
    bool good_input_;

    // input flag
    int flag_;

    
    /**
     * \brief Show help
     */
    static void help() {
        cout << "Camera calibration test tool v0.1.\n"
            << "Usage: calibration configurationFile\n"
            << "Near the sample file you'll find the configuration file, which has detailed help of "
            "how to edit it.  It may be any OpenCV supported file format XML/YAML/JSON." << endl;
    }

    /**
     * \brief Computes the reprojection errors
     * \param object_points 
     * \param image_points 
     * \param rvecs 
     * \param tvecs 
     * \param camera_matrix 
     * \param dist_coeffs 
     * \param per_view_errors 
     * \return The error ratio as double precision floating value
     */
    static double compute_reprojection_errors(const vector<vector<cv::Point3f>>& object_points,
                                              const vector<vector<cv::Point2f>>& image_points,
                                              const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
                                              const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                                              vector<float>& per_view_errors) {
        vector<cv::Point2f> image_points_2;
        auto total_points = 0;
        double total_err = 0;
        per_view_errors.resize(object_points.size());

        for (auto i = 0; i < static_cast<int>(object_points.size()); ++i) {
            projectPoints(cv::Mat(object_points[i]), rvecs[i], tvecs[i], camera_matrix, dist_coeffs, image_points_2);
            auto err = norm(cv::Mat(image_points[i]), cv::Mat(image_points_2), CV_L2);

            auto n = static_cast<int>(object_points[i].size());
            per_view_errors[i] = static_cast<float>(std::sqrt(err * err / n));
            total_err += err * err;
            total_points += n;
        }
        return cv::sqrt(total_err / total_points);
    }

    // Print camera parameters to the output file
    void save_camera_params(Settings& s, cv::Size& image_size, cv::Mat& camera_matrix, cv::Mat& dist_coeffs,
                            const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
                            const vector<float>& reproj_errs, const vector<vector<cv::Point2f>>& image_points,
                            double total_avg_err) const {
        cv::FileStorage fs(s.output_file_name_, cv::FileStorage::WRITE_BASE64);

        fs << "calibration_Time" << get_time_date();

        if (!rvecs.empty() || !reproj_errs.empty()) {
            fs << "nrOfFrames" << static_cast<int>(max(rvecs.size(), reproj_errs.size()));
        }
        fs << "image_Width" << image_size.width;
        fs << "image_Height" << image_size.height;
        fs << "board_Width" << s.board_size_.width;
        fs << "board_Height" << s.board_size_.height;
        fs << "square_Size" << s.square_size_;

        if (s.flag_ & CV_CALIB_FIX_ASPECT_RATIO) {
            fs << "FixAspectRatio" << s.aspect_ratio_;
        }

        if (s.flag_) {
            string out("flags: ");
            if (s.flag_ & CV_CALIB_USE_INTRINSIC_GUESS) {
                out += " +use_intrinsic_guess";
            }
            if (s.flag_ & CV_CALIB_FIX_ASPECT_RATIO) {
                out += " +fix_aspectRatio";
            }
            if (s.flag_ & CV_CALIB_FIX_PRINCIPAL_POINT) {
                out += " +fix_principal_point";
            }
            if (s.flag_ & CV_CALIB_ZERO_TANGENT_DIST) {
                out += " +zero_tangent_dist";
            }
            cvWriteComment(*fs, out.c_str(), 0);

            //char buf[1024];
            //sprintf(buf, "flags: %s%s%s%s",
            //        s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            //        s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            //        s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            //        s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
            //cvWriteComment(*fs, buf, 0);

        }

        fs << "flagValue" << s.flag_;

        fs << "Camera_Matrix" << camera_matrix;
        fs << "Distortion_Coefficients" << dist_coeffs;

        fs << "Avg_Reprojection_Error" << total_avg_err;
        if (!reproj_errs.empty()) {
            fs << "Per_View_Reprojection_Errors" << cv::Mat(reproj_errs);
        }

        if (!rvecs.empty() && !tvecs.empty()) {
            CV_Assert(rvecs[0].type() == tvecs[0].type());
            cv::Mat bigmat(static_cast<int>(rvecs.size()), 6, rvecs[0].type());
            for (auto i = 0; i < static_cast<int>(rvecs.size()); i++) {
                // ReSharper disable CppInitializedValueIsAlwaysRewritten
                auto rotation = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
                auto translation = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));
                // ReSharper restore CppInitializedValueIsAlwaysRewritten

                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                rotation = rvecs[i].t();
                translation = tvecs[i].t();
            }
            cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
            fs << "Extrinsic_Parameters" << bigmat;
        }

        if (image_points.empty()) {
            return;
        }

        cv::Mat pimage_mat(static_cast<int>(image_points.size()), static_cast<int>(image_points[0].size()), CV_32FC2);
        for (auto i = 0; i < static_cast<int>(image_points.size()); i++) {
            auto r = pimage_mat.row(i).reshape(2, pimage_mat.cols);
            cv::Mat imgpti(image_points[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << pimage_mat;
    }

    bool run_calibration(Settings& s, cv::Size& image_size, cv::Mat& camera_matrix, cv::Mat& dist_coeffs,
                         vector<vector<cv::Point2f>> image_points, vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs,
                         vector<float>& reproj_errs, double& total_avg_err) const {

        log_time << "Configuring matricies.." << endl;

        camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        if (s.flag_ & CV_CALIB_FIX_ASPECT_RATIO) {
            camera_matrix.at<double>(0, 0) = 1.0;
        }

        dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

        log_time << "Calculating board corner positions and resizing object points..\n";

        vector<vector<cv::Point3f>> object_points(1);
        compute_board_corner_positions(s.board_size_, s.square_size_, object_points[0], s.calibration_pattern_);

        object_points.resize(image_points.size(), object_points[0]);

        log_time << "Configuring camera intrinsic & extrinsic parameters." << endl;

        //Find intrinsic and extrinsic camera parameters
        auto rms = calibrateCamera(object_points, image_points, image_size, camera_matrix,
                                   dist_coeffs, rvecs, tvecs, s.flag_ | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

        log_time << "Re-projection error reported by calibrateCamera: " << rms << endl;

        auto ok = checkRange(camera_matrix) && checkRange(dist_coeffs);

        total_avg_err = compute_reprojection_errors(object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs, reproj_errs);

        return ok;
    }

    static void compute_board_corner_positions(cv::Size board_size, float square_size, vector<cv::Point3f>& corners, Pattern pattern_type /*= Settings::CHESSBOARD*/) {
        corners.clear();

        switch (pattern_type) {
        case Pattern::CHESSBOARD:
        case Pattern::CIRCLES_GRID:
            for (auto i = 0; i < board_size.height; ++i) {
                for (auto j = 0; j < board_size.width; ++j) {
                    corners.emplace_back(cv::Point3f(float(j * square_size), float(i * square_size), 0));
                }
            }
            break;
        case Pattern::ASYMMETRIC_CIRCLES_GRID:
            for (auto i = 0; i < board_size.height; i++) {
                for (auto j = 0; j < board_size.width; j++) {
                    corners.emplace_back(cv::Point3f(float((2 * j + i % 2) * square_size), float(i * square_size), 0));
                }
            }
            break;
        default:
            break;
        }
    }

    bool run_calibration_and_save(Settings& s, cv::Size image_size, cv::Mat& camera_matrix, cv::Mat& dist_coeffs, vector<vector<cv::Point2f>> image_points) const {
        vector<cv::Mat> rvecs;
        vector<cv::Mat> tvecs;
        vector<float> reprojErrs;
        double totalAvgErr = 0;

        auto ok = run_calibration(s, image_size, camera_matrix, dist_coeffs, image_points, rvecs, tvecs, reprojErrs, totalAvgErr);
        std::string msg = cv::format("Calibration %s. avg re projection error = %f\n", (ok ? "succeeded" : "failed"), totalAvgErr);

        if (ok) {
            log_time << msg;
            save_camera_params(s, image_size, camera_matrix, dist_coeffs, rvecs, tvecs, reprojErrs, image_points, totalAvgErr);
        }

        log_err << msg;

        return ok;
    }

    int init(Settings& s, const string inputSettingsFile = "default.xml") const {
        help();

        // Read the settings
        cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            log_err << __FUNCTION__ << " Could not open the configuration file: \"" << inputSettingsFile << "\"\n";
            return -1;
        }

        fs["Settings"] >> s;
        // close Settings file
        fs.release();

        if (!s.good_input_) {
            log_err << __FUNCTION__ << " Invalid input detected. Stopping calibration.\n";
            return -1;
        }

        vector<vector<cv::Point2f>> image_points;
        cv::Mat camera_matrix, dist_coeffs;
        cv::Size image_size;
        auto mode = s.input_type_ == InputType::IMAGE_LIST ? Mode::CAPTURING : Mode::DETECTION;
        clock_t prev_timestamp = 0;
        const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);
        const char ESC_KEY = 27;

        cv::namedWindow("Image View", cv::WINDOW_FREERATIO);

        const auto def_zero_zone = cv::Size(-1, -1);
        const auto def_win_size = cv::Size(11, 11);

        for (auto i = 0;; ++i) {
            auto view = s.next_image();
            auto blink_output = false;

            //-----  If no more image, or got enough, then stop calibration and show result -------------
            if (mode == Mode::CAPTURING && image_points.size() >= static_cast<unsigned>(s.num_frames_)) {
                mode = tg::iif(s.run_calibration_and_save(s, image_size, camera_matrix, dist_coeffs, image_points), Mode::CALIBRATED, Mode::DETECTION);
            }

            // If no more images then run calibration, save and stop loop.
            if (view.empty()) {
                if (image_points.size() > 0) {
                    s.run_calibration_and_save(s, image_size, camera_matrix, dist_coeffs, image_points);
                }
                break;
            }

            // Format input image.
            image_size = view.size();

            if (s.flip_vertical_) {
                flip(view, view, 0);
            }

            vector<cv::Point2f> point_buf;

            // Find feature points on the input format
            bool found;
            switch (s.calibration_pattern_) {
            case Pattern::CHESSBOARD:
                found = findChessboardCorners(view, s.board_size_, point_buf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
                break;
            case Pattern::CIRCLES_GRID:
                found = findCirclesGrid(view, s.board_size_, point_buf);
                break;
            case Pattern::ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid(view, s.board_size_, point_buf, cv::CALIB_CB_ASYMMETRIC_GRID);
                break;
            default:
                found = false;
                break;
            }

            // If done with success,
            if (found) {
                // improve the found corners' coordinate accuracy for chessboard
                if (s.calibration_pattern_ == Pattern::CHESSBOARD) {
                    // from colour to gray
                    cv::Mat view_gray;
                    cvtColor(view, view_gray, cv::COLOR_BGR2GRAY);
                    cornerSubPix(view_gray, point_buf, def_win_size, def_zero_zone, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                }

                // For camera only take new samples after delay time
                if (mode == Mode::CAPTURING && (!s.input_capture_.isOpened() || clock() - prev_timestamp > s.delay_ * 1e-3 * CLOCKS_PER_SEC)) {
                    image_points.emplace_back(point_buf);
                    prev_timestamp = clock();
                    blink_output = s.input_capture_.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners(view, s.board_size_, cv::Mat(point_buf), found);
            }

            //----------------------------- Output Text ------------------------------------------------
            string msg = (mode == Mode::CAPTURING) ? "100/100" : mode == Mode::CALIBRATED ? "Calibrated" : "Press 'g' to start";
            auto base_line = 0;
            auto text_size = cv::getTextSize(msg, 1, 1, 1, &base_line);
            cv::Point text_origin(view.cols - 2 * text_size.width - 10, view.rows - 2 * base_line - 10);

            if (mode == Mode::CAPTURING) {
                msg = cv::format(tg::iif(s.show_undistorsed_, "%d/%d Undist", "%d/%d"), static_cast<int>(image_points.size()), s.num_frames_);
            }

            putText(view, msg, text_origin, 1, 1, mode == Mode::CALIBRATED ? GREEN : RED);

            if (blink_output) {
                bitwise_not(view, view);
            }

            //------------------------- Video capture  output  undistorted ------------------------------
            if (mode == Mode::CALIBRATED && s.show_undistorsed_) {
                auto temp = view.clone();
                undistort(temp, view, camera_matrix, dist_coeffs);
            }

            //------------------------------ Show image and check for input commands -------------------
            log_time << s.image_list_[s.at_image_list_] << endl;
            imshow("Image View", view);
            auto key = static_cast<char>(cv::waitKey(s.input_capture_.isOpened() ? 50 : s.delay_));

            if (key == ESC_KEY) {
                break;
            }

            if (key == 'u' && mode == Mode::CALIBRATED) {
                tg::flip_bool(s.show_undistorsed_);
            }

            if (s.input_capture_.isOpened() && key == 'g') {
                mode = Mode::CAPTURING;
                image_points.clear();
            }
        }

        // -----------------------Show the undistorted image for the image list ------------------------
        if (s.input_type_ == InputType::IMAGE_LIST && s.show_undistorsed_) {
            cv::Mat rview, map1, map2;
            initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 1, image_size, nullptr), image_size, CV_16SC2, map1, map2);

            for (auto i = 0; i < static_cast<int>(s.image_list_.size()); i++) {
                auto view = cv::imread(s.image_list_[i], 1);
                if (view.empty()) {
                    continue;
                }

                remap(view, rview, map1, map2, cv::INTER_LINEAR);
                imshow("Image View", rview);
                auto c = static_cast<char>(cv::waitKey());
                if (c == ESC_KEY || c == 'q' || c == 'Q') {
                    break;
                }
            }
        }

        return 0;
    }


private:
    string pattern_to_use_;
};

static void read(const cv::FileNode& node, Settings& x, const Settings& default_value = Settings()) {
    if (node.empty()) {
        x = default_value;
        return;
    }
    x.read(node);
}

//bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
//	vector<vector<Point2f> > imagePoints);


//int Settings::readSettings(const string calibrationFile, Mat *cameraMatrix, Mat *distortion, double *avg_error, Mat *perViewReprojectionErrors, Mat *extrinsicParameters, Mat *imagePoints)
//{
//	FileStorage fs;
//	fs.open(calibrationFile, FileStorage::READ);
//
//	if (!fs.isOpened())
//	{
//		cerr << "Failed to open " << calibrationFile << endl;
//		return -1;
//	}
//
//	//cv::FileNode d = fs["data"];
//	fs["Camera_Matrix"] >> *cameraMatrix;
//
//	fs["Distortion_Coefficients"] >> *distortion;
//
//	fs["Avg_Reprojection_Error"] >> *avg_error;
//
//	fs["Per_View_Reprojection_Errors"] >> *perViewReprojectionErrors;
//	fs["Extrinsic_Parameters"] >> *extrinsicParameters;
//
//	fs["Image_points"] >> *imagePoints;
//
//	fs.release();
//
//	return 0;
//}
