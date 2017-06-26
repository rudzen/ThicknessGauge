#include "Calib2.h"
#include "Util/Ztring.h"

void Calib2::show_begin() {
    
    std::cout << "Rudz calibration wizard initiated..\n";
    std::cout << "-----------------------------------\n";
    std::cout << "At any time, press ctrl-c to abord!\n\n";

}

bool Calib2::init() {

    if (pcapture == nullptr) {
        // run the basic process for the capture object
        pcapture = std::make_unique<CapturePvApi>();
    } else {
        // double check for weirdness
        if (pcapture->is_open()) {
            pcapture->close();
        }
    }

    // always perform complete re-init.

    auto capture_device_ok = pcapture->initialize();

    if (!capture_device_ok) {
        log_time << "Capture device could not be initialized, aborting.\n";
        return false;
    }

    //initVideoCapture(); // for opencv capture object

    capture_device_ok = pcapture->open();

    if (!capture_device_ok) {
        log_time << "Capture device could not be openened, aborting.\n";
        pcapture->initialized(false);
        return false;
    }

    //capture->print_attr();

    pcapture->pixel_format();

    pcapture->reset_binning();

    pcapture->packet_size(8228);

    auto ok = pcapture->frame_init();
    if (!ok)
        return false;

    ok = pcapture->cap_init();
    if (!ok)
        return false;

    ok = pcapture->aquisition_init();
    if (!ok)
        return false;

    ok = pcapture->region(pcapture->default_roi_full);
    if (!ok)
        return false;

    return true;

}

int Calib2::calib() {

    show_begin();

    if (!init()) {
        log_time << cv::format("Calibration init failed.\n");
        return -1;
    }

    log_time << cv::format("Calibration init ok.\n");

    Ztring in;

    log_time << "(1 /  ) Enter number of squares (horizontal) >";
    std::getline(std::cin, in);
    board_w = Ztring::stoi(in);
    log_time << "(2 /  ) Enter number of corners (vertical) >";
    std::getline(std::cin, in);
    board_h = Ztring::stoi(in);
    log_time << "(3 /  ) Enter number of boards to process (0 to exit) >";
    std::getline(std::cin, in);
    n_boards = Ztring::stoi(in);
    if (n_boards == 0)
        return -2;

    auto board_n = board_w * board_h;
    auto board_sz = cv::Size(board_w, board_h);

    // ALLOCATE STORAGE
    //
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    // Capture corner views: loop until we've got n_boards successful

    // captures (all corners on the board are found).
    //
    double last_captured_timestamp = 0;
    cv::Size image_size;
    while (image_points.size() < static_cast<size_t>(n_boards)) {
        cv::Mat image0, image;
        pcapture->cap_single(image0);
        image_size = image0.size();
        cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);
        // Find the board
        //
        std::vector<cv::Point2f> corners;
        auto found = cv::findChessboardCorners(image, board_sz, corners);
        // Draw it
        //
        drawChessboardCorners(image, board_sz, corners, found);
        // If we got a good board, add it to our data
        //
        double timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;
        if (found && timestamp - last_captured_timestamp > 1) {
            last_captured_timestamp = timestamp;
            image ^= cv::Scalar::all(255);
            cv::Mat mcorners(corners); // do not copy the data
            mcorners *= (1. / image_sf); // scale the corner coordinates
            image_points.push_back(corners);
            object_points.push_back(std::vector<cv::Point3f>());
            std::vector<cv::Point3f>& opts = object_points.back();
            opts.resize(board_n);
            for (int j = 0; j < board_n; j++) {
                opts[j] = cv::Point3f(static_cast<float>(j / board_w), static_cast<float>(j % board_w), 0.f);
            }
            std::cout << "Collected our " << static_cast<int>(image_points.size()) <<
                " of " << n_boards << " needed chessboard images\n" << std::endl;
        }
        cv::imshow("Calibration", image); //show in color if we did collect the image
        if ((cv::waitKey(30) & 255) == 27)
            return -1;
    }
    // END COLLECTION WHILE LOOP.

    cv::destroyWindow("Calibration");
    std::cout << "\n\n*** CALIBRATING THE CAMERA...\n" << std::endl;

    // CALIBRATE THE CAMERA!
    //
    cv::Mat intrinsic_matrix, distortion_coeffs;
    double err = cv::calibrateCamera(object_points, image_points, image_size, intrinsic_matrix, distortion_coeffs, cv::noArray(), cv::noArray(), cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);
    // SAVE THE INTRINSICS AND DISTORTIONS
    std::cout << " *** DONE!\n\nReprojection error is " << err <<
        "\nStoring Intrinsics.xml and Distortions.xml files\n\n";
    cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
    fs << "image_width" << image_size.width << "image_height" << image_size.height
        << "camera_matrix" << intrinsic_matrix << "distortion_coefficients"
        << distortion_coeffs;
    fs.release();
    // EXAMPLE OF LOADING THESE MATRICES BACK IN:
    fs.open("intrinsics.xml", cv::FileStorage::READ);
    std::cout << "\nimage width: " << static_cast<int>(fs["image_width"]);
    std::cout << "\nimage height: " << static_cast<int>(fs["image_height"]);
    cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
    fs["camera_matrix"] >> intrinsic_matrix_loaded;
    fs["distortion_coefficients"] >> distortion_coeffs_loaded;
    std::cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
    std::cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << std::endl;
    // Build the undistort map which we will use for all
    // subsequent frames.
    //
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(intrinsic_matrix_loaded, distortion_coeffs_loaded, cv::Mat(), intrinsic_matrix_loaded, image_size, CV_16SC2, map1, map2);
    // Just run the camera to the screen, now showing the raw and
    // the undistorted image.
    //
    cv::Mat image, image0;
    for (;;) {
        pcapture->cap_single(image0);
        if (image0.empty())
            break;
        cv::remap(image0, image, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        cv::imshow("Undistorted", image);
        if ((cv::waitKey(30) & 255) == 27)
            break;
    }

    pcapture->close();
    pcapture->uninitialize();

    return 0;
}
