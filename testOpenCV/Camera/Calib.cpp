#include "Calib.h"
#include "../namespaces/tg.h"
#include "Util/Ztring.h"

using namespace tg;

void Calib::show_begin() {

    log_time << "Rudz calibration wizard v1 initiated..\n";

}

void Calib::save_data(std::string& filename) const {

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "calibration_Time" << get_time_date();
    fs << "image_width" << image_.cols;
    fs << "image_height" << image_.rows;
    fs << "horizontal_squares" << corners_horizontal_;
    fs << "vertical_squares" << corners_vertical_;
    if (!rvecs_.empty()) {
        fs << "rvecs" << rvecs_;
    }
    if (!tvecs_.empty()) {
        fs << "tvecs" << tvecs_;
    }

    fs << "intrinsic" << intrinsic_;
    fs << "dist_coeffs" << dist_coeffs_;

    fs.release();

}

void Calib::run_calib() {

    show_begin();

    Ztring in;

    log_time << "(1 /  ) Enter number of squares (horizontal) >";
    std::getline(std::cin, in);
    corners_horizontal_ = Ztring::stoi(in);
    log_time << "(2 /  ) Enter number of corners (vertical) >";
    std::getline(std::cin, in);
    corners_vertical_ = Ztring::stoi(in);
    log_time << "(3 /  ) Enter number of boards to process (0 to exit) >";
    std::getline(std::cin, in);
    board_count_ = Ztring::stoi(in);
    if (board_count_ == 0) {
        return;
    }

    auto board_sz = cv::Size(corners_horizontal_, corners_vertical_);
    auto num_squares = board_sz.area();

    log_time << cv::format("Processing... square count = %i.\n", num_squares);

    auto capture = cv::VideoCapture(0);

    auto successes = 0;

    capture >> image_;

    log_time << "Generating objects..\n";

    obj_.reserve(num_squares);

    for (auto j = 0; j < num_squares; j++) {
        obj_.emplace_back(cv::Point3f(j / static_cast<float>(corners_horizontal_), static_cast<float>(j % corners_horizontal_), 0.0f));
    }

    corners_.reserve(num_squares);
    image_points_.reserve(num_squares);
    object_points_.reserve(num_squares);

    while (successes < board_count_) {
        log_time << cv::format("Searching board %i of %i. (esc = exit, any other key to proceed to next board)\n", successes + 1, board_count_);

        cv::cvtColor(image_, gray_image_, CV_BGR2GRAY);
        auto found = findChessboardCorners(image_, board_sz, corners_, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        if (found) {
            cornerSubPix(gray_image_, corners_, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray_image_, board_sz, corners_, found);
        }

        imshow("win1", image_);
        imshow("win2", gray_image_);
        capture >> image_;
        auto key = cv::waitKey(1);

        if (key == 27) {
            return;
        } else if (key == ' ' && found != 0) {
            image_points_.emplace_back(corners_);
            object_points_.emplace_back(obj_);
            log_time << "Data stored..\n";
            successes++;
            if (successes >= board_count_) {
                break;
            }
        }
    }

    log_time << "Calibrating....\n";

    intrinsic_ = cv::Mat(3, 3, CV_32FC1);

    intrinsic_.ptr<float>(0)[0] = 1;
    intrinsic_.ptr<float>(1)[1] = 1;

    calibrateCamera(object_points_, image_points_, image_.size(), intrinsic_, dist_coeffs_, rvecs_, tvecs_);

    cv::Mat image_undistorted;

    log_time << "Undistorting...\n";

    capture >> image_;
    capture.release();
    undistort(image_, image_undistorted, intrinsic_, dist_coeffs_);
    imshow("win1", image_);
    cv::imshow("win2", image_undistorted);

    log_time << "Enter filename to save calibration data to (ctrl-c to abort) >";

    std::getline(std::cin, in);
    if (in.empty()) {
        return;
    }

    save_data(in);

}
