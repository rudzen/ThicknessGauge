#include <algorithm>
#include <array>
#include <opencv2/core/base.hpp>

#include "ThicknessGauge.h"

#include "IO/ImageSave.h"
#include "IO/GlobGenerator.h"

#include "UI/ProgressBar.h"

#include "Histogram/Histogram.h"
#include "Histogram/HistoPeak.h"

#include "CV/CannyR.h"
#include "CV/HoughLinesR.h"
#include "CV/FilterR.h"
#include "CV/HoughLinesPR.h"
#include "CV/SparseR.h"

#include "Exceptions/ThrowAssert.h"
#include "Exceptions/CaptureFailException.h"


#include "Camera/OpenCVCap.h"

#include "namespaces/tg.h"
#include "namespaces/filesystem.h"
#include "namespaces/stl.h"
#include "namespaces/filters.h"
#include "namespaces/sort.h"
#include "namespaces/validate.h"
#include "namespaces/cvR.h"
#include "namespaces/draw.h"

using namespace tg;

/**
 * \brief Initializes the class and loads any aditional information
 * \param glob_name if "camera", use camera, otherwise load from glob folder
 */
bool ThicknessGauge::initialize(std::string& glob_name) {
    data->globName = glob_name;
    //addNulls();

    // determin where to get the frames from.
    if (glob_name == "camera") {

        if (capture == nullptr) {
            // run the basic process for the capture object
            capture = std::make_unique<CapturePvApi>();
        } else {
            // double check for weirdness
            if (capture->isOpen()) {
                capture->close();
            }

            //capture->initialized(false);
        }

        // always perform complete re-init.

        auto capture_device_ok = capture->initialize();

        if (!capture_device_ok) {
            log_time << "Capture device could not be initialized, aborting.\n";
            return false;
        }

        //initVideoCapture(); // for opencv capture object

        capture_device_ok = capture->open();

        if (!capture_device_ok) {
            log_time << "Capture device could not be openened, aborting.\n";
            capture->initialized(false);
            return false;
        }

        capture->reset();

        for (auto& fs : frameset)
            capture->capture(25, fs->frames, fs->exp_ms);

        capture->close();

        capture->uninitialize();

        //captureFrames(0, frameCount_, 5000);
    } else
        loadGlob(glob_name);

    return
        true;
}

/**
 * \brief Initializes the capture device using PV_API constant
 * (requires that OpenCV is compiled with the location of the PvAPI, deprecated version)
 */
void ThicknessGauge::initVideoCapture() {


    //capture = std::make_unique<OpenCVCap>();
    //if (!capture->cap.open(CV_CAP_PVAPI)) {
    //    sync_cout << "Failed to open using PV_API, attempting defatult";
    //    if (!capture->cap.open(CV_CAP_ANY)) {
    //        CV_Error(cv::Error::StsError, "Error while attempting to open capture device.");
    //    }
    //}

    //capture->targetStdDev(63.0);
    //capture->deltaValue(2.0);
    //log_time << "whoop: " << capture->detectExposure() << endl;

}

/**
 * \brief Initializes the calibration settings
 * \param fileName The filename for the calibration settings
 */
void ThicknessGauge::initCalibrationSettings(string fileName) const {
    cs->readSettings(fileName);
}

/**
 * \brief Generates a custom glob
 * \param name The name of the glob, let it be a valid foldername!!!
 */
void ThicknessGauge::generateGlob(std::string& name) {
    initVideoCapture();

    file::create_directory(name);
    auto pb_title = "Capturing glob " + name;

    ProgressBar pb(frameCount_, pb_title.c_str());
    pb.SetFrequencyUpdate(10);
    pb.SetStyle("-", " ");

    cv::Mat t;

    cv::VideoCapture cap;
    cap.open(CV_CAP_PVAPI);

    unsigned long progress = 0;
    for (auto i = 0; i < frameCount_; ++i) {
        pb.Progressed(++progress);
        cap >> t;
        cv::imwrite(name + "/img" + to_string(i) + ".png", t);
    }
    cap.release();
    pb.Progressed(frameCount_);
}

/**
 * \brief Determins the marking boundries
 * \return 2 Float vector with the points marking the boundries as pair, where first = left, second = right
 */
void ThicknessGauge::computeMarkingHeight() {

    try {

        // test print for all framesets
        for (auto& f: frameset) {
            f->compute();
            //log_time << f << endl;
        }

        uint64 time_start = cv::getTickCount();

        // configure frames based on center vertical splitting of the original frames
        //vector<cv::Mat> leftFrames(frameCount_);
        //vector<cv::Mat> rightFrames(frameCount_);
        //splitFrames(leftFrames, rightFrames);

        // configure filters for show window
        filter_marking->setShowWindows(showWindows_);
        filter_baseline->setShowWindows(showWindows_);

        // houghlines to determin where the actual marking is in the frame
        auto hough_vertical = make_shared<HoughLinesR>(1, static_cast<const int>(calc::DEGREES), 40, showWindows_);

        // configure the diffrent functionalities
        hough_vertical->setAngleLimit(30);
        hough_vertical->setShowWindows(showWindows_);

        data->markingRect = computerMarkingRectangle(hough_vertical);
        hough_vertical->setMarkingRect(data->markingRect);

        // check the resulting rectangle for weirdness
        if (data->markingRect.x < 0.0 || data->markingRect.y < 0.0 || data->markingRect.width > imageSize_.width || data->markingRect.height > imageSize_.height || data->markingRect.area() >= imageSize_.area()) {
            CV_Error(cv::Error::StsBadSize, cv::format("Marking rectangle has bad size : [x:%f] [y:%f] [w:%f] [h:%f]\n", data->markingRect.x, data->markingRect.y, data->markingRect.width, data->markingRect.height));
        }

        // make sure the minimum is at least 10 pixels.
        auto min_line_len = computeHoughPMinLine(10.0, data->markingRect);

        // horizontal houghline extension class
        auto hough_horizontal = make_shared<HoughLinesPR>(1, cvRound(calc::DEGREES), 40, cvRound(min_line_len), showWindows_);

        hough_horizontal->setMaxLineGab(12);
        hough_horizontal->setMarkingRect(data->markingRect);
        hough_horizontal->setShowWindows(showWindows_);

        // morph extension class
        auto morph = make_shared<MorphR>(cv::MORPH_GRADIENT, 1, showWindows_);

        computeBaseLineAreas(hough_horizontal, morph);
        //std::cout << cv::format("Base line Y [left] : %f\n", data->baseLines[1]);
        //std::cout << cv::format("Base line Y [right]: %f\n", data->baseLines[3]);

        // compute the intersection points based on the borders of the markings and the baseline for the laser outside the marking
        calc::compute_intersection_points(data->baseLines, hough_vertical->getLeftBorder(), hough_vertical->getRightBorder(), data->intersections);

        // grabs the in between parts and stores the data
        //computerInBetween(filter_baseline, hough_horizontal, morph);

        log_time << "intersection points: " << data->intersections << endl;

        // pixel cut off is based on the border of the marking..
        auto intersect_cutoff = calc::compute_intersection_cut(hough_vertical->getLeftBorder(), hough_vertical->getRightBorder());

        data->intersectionCuts[0] = data->intersections[0] - intersect_cutoff[0];
        data->intersectionCuts[3] = data->intersections[3] + intersect_cutoff[1];

        // set data for in-between areas
        data->middlePieces[0] = data->intersectionCuts[0];
        data->middlePieces[1] = data->intersectionCuts[3] - data->intersectionCuts[0];

        if (!validate::validate_rect(data->markingRect)) {
            CV_Error(cv::Error::BadROISize, "Marking rectangle dimensions are fatal.");
        }

        // adjust the baselines according to the intersection points. (could perhaps be useful in the future)
        cvr::adjust_marking_rect(data->markingRect, data->intersections, intersect_cutoff[0]);

        // testing angles between baseline.. should be max 5 degrees
        cv::Point2d line_left(data->markingRect.x, data->baseLines[1]);
        cv::Point2d line_right(line_left.x + data->markingRect.width, data->baseLines[3]);

        auto angle = calc::angle_between_lines(line_left, line_right);
        log_time << cv::format("Angle between baselines: [r/d] = [%f/%f]\n", angle, calc::rad_to_deg(angle));

        //std::cout << cv::format("Adjusted marking rect: [x: %f | y: %f | w: %f | h: %f]\n", data->markingRect.x, data->markingRect.y, data->markingRect.width, data->markingRect.height);
        //std::cout << cv::format("Adjusted base line Y [left] : %f\n", data->baseLines[1]);
        //std::cout << cv::format("Adjusted base line Y [right]: %f\n", data->baseLines[3]);

        // filter for laser detection
        auto filter_laser = make_shared<FilterR>("Laser Filter", showWindows_);
        filter_laser->setShowWindows(showWindows_);

        // main laser class
        auto laser = make_shared<LaserR>();

        // computes the Y locations of the laserline inside the marking rect
        computeLaserLocations(laser, filter_laser);

        if (showWindows_)
            draw::removeAllWindows();

        // do a quick pass of validation before modifying the data
        validate::valid_data(data);

        auto adjust_points = [&](cv::Point2d& p) {
            p.y = imageSize_.height - p.y;
        };

        std::for_each(data->leftPoints.begin(), data->leftPoints.end(), adjust_points);
        std::for_each(data->rightPoints.begin(), data->rightPoints.end(), adjust_points);
        std::for_each(data->centerPoints.begin(), data->centerPoints.end(), adjust_points);

        uint64 time_end = cv::getTickCount();

        frameTime_ = static_cast<double>((time_end - time_start) / cv::getTickFrequency());

        log_time << "Total compute time (seconds) : " << frameTime_ << endl;

        if (draw::is_escape_pressed(30))
            return;

    } catch (cv::Exception& e) {
        cerr << cv::format("Exception caught in computeMarkingHeight().\n%s\n", e.msg.c_str());
    }

}

/**
 * \brief Computes the base line areas and determine the actual base line.
 * \param hough The houghlines class
 * \param morph The morphology class
 */
void ThicknessGauge::computeBaseLineAreas(shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) {

    filter_baseline->setKernel(filters::kernel_line_left_to_right);

    morph->setMethod(cv::MORPH_GRADIENT);
    morph->setIterations(1);

    const std::string window_left = "test baseline left";
    const std::string window_right = "test baseline right";

    if (showWindows_) {
        draw::makeWindow(window_left);
        draw::makeWindow(window_right);
    }

    auto quarter = static_cast<double>(imageSize_.height) / 4.0;
    auto base_line_y = imageSize_.height - quarter;

    auto marking = hough->getMarkingRect();

    cv::Rect2d left_baseline;
    left_baseline.x = 0.0;
    left_baseline.y = base_line_y;
    left_baseline.width = marking.x;
    left_baseline.height = quarter;

    if (!validate::validate_rect(left_baseline)) {
        log_time << "Validation error for left_baseline in computeBaseLineAreas()." << endl;
    }

    cv::Rect2d right_baseline;
    right_baseline.x = marking.x + marking.width;
    right_baseline.y = base_line_y;
    right_baseline.width = imageSize_.width - right_baseline.x;
    right_baseline.height = quarter;

    if (!validate::validate_rect(right_baseline)) {
        log_time << "Validation error for right_baseline in computeBaseLineAreas()." << endl;
    }

    // cannot be resized
    vector<cv::Mat> left_frames;
    vector<cv::Mat> right_frames;

    //frame pointer for the desired frame set
    unsigned int frame_index = 2;

    auto frames = frameset[frame_index].get();

    log_time << cv::format("computeBaseLineAreas using exposure set %i : %s (%i)\n", frame_index, frames->exp_ext, frames->exp_ms);

    // generate baseline images..
    for (auto i = frameCount_; i--;) {
        left_frames.emplace_back(frames->frames[i](left_baseline));
        right_frames.emplace_back(frames->frames[i](right_baseline));
    }

    auto left_size = left_frames.front().size();
    auto left_cutoff = left_size.width / 2.0;
    auto right_size = right_frames.front().size();
    auto right_cutoff = right_size.width / 2.0;

    auto left_y = 0.0;
    auto right_y = 0.0;

    std::vector<cv::Point2f> left_elements(left_size.area());
    std::vector<cv::Point2f> right_elements(right_size.area());

    auto offset_y = imageSize_.height - quarter;

    auto left_avg = 0.0;
    auto right_avg = 0.0;

    auto running = true;

    while (running) {

        left_elements.clear();
        right_elements.clear();

        left_y = 0.0;
        right_y = 0.0;

        left_avg = 0.0;
        right_avg = 0.0;

        cv::Mat org;

        // left

        for (auto& left : left_frames) {
            org = left.clone();
            auto h = left.clone();
            hough->setOriginal(h);

            processMatForLine(org, hough, morph);

            const auto& lines = hough->getRightLines(); // inner most side
            for (auto& line : lines) {
                if (line.entry[0] > left_cutoff) {
                    stl::copyVector(line.elements, left_elements);
                }
            }

            if (draw::is_escape_pressed(30))
                running = false;

        }

        // generate real boundry
        auto left_boundry = cv::minAreaRect(left_elements);
        auto left_boundry_rect = left_boundry.boundingRect();

        log_time << "left_boundry_rect: " << left_boundry_rect.y << endl;

        left_boundry_rect.width -= 40;

        if (showWindows_) {
            draw::drawRectangle(org, left_boundry_rect, cv::Scalar(255, 255, 255));
            draw::showImage(window_left, org);
            if (draw::is_escape_pressed(30))
                running = false;
        }

        auto t = org(left_boundry_rect);
        left_y = static_cast<double>(left_boundry_rect.y);
        left_y += offset_y;
        left_y += calc::real_intensity_line(t, data->leftPoints, t.rows, 0);

        log_time << "left baseline: " << left_y << endl;

        // right

        for (auto& right : right_frames) {
            org = right.clone();
            auto h1 = right.clone();
            hough->setOriginal(h1);

            processMatForLine(org, hough, morph);

            const auto& lines = hough->getLeftLines(); // inner most side
            for (auto& h : lines) {
                if (h.entry[2] < right_cutoff)
                    stl::copyVector(h.elements, right_elements);
            }

            if (draw::is_escape_pressed(30))
                running = false;

        }

        // generate real boundry
        auto right_boundry = cv::minAreaRect(right_elements);
        auto right_boundry_rect = right_boundry.boundingRect();

        right_boundry_rect.x += 40;

        if (showWindows_) {
            draw::drawRectangle(org, right_boundry_rect, cv::Scalar(255, 255, 255));
            draw::showImage(window_right, org);
            if (draw::is_escape_pressed(30))
                running = false;
        }

        log_time << "right_boundry_rect: " << right_boundry_rect.y << endl;

        t = org(right_boundry_rect);
        right_y = static_cast<double>(right_boundry_rect.y);
        right_y += calc::real_intensity_line(t, data->rightPoints, t.rows, 0);
        right_y += offset_y;

        log_time << "right baseline: " << right_y << endl;

        // forcefully break out of the loop
        if (!showWindows_)
            break;

        running = false;
    }

    data->baseLines[0] = 0.0;
    data->baseLines[1] = left_y;
    data->baseLines[2] = 0.0;
    data->baseLines[3] = right_y;

    if (!validate::valid_vec(data->baseLines)) {
        log_time << "Validation error for data->baseLines in computeBaseLineAreas()." << endl;
    }

    if (showWindows_) {
        draw::removeWindow(window_left);
        draw::removeWindow(window_right);
    }
}

/**
 * \brief Processes the matrix for optimal output and computes the line information based on the results
 * \param org The matrix to perform the process on
 * \param hough The hough extension class used
 * \param morph The morphology extenstion class used
 */
void ThicknessGauge::processMatForLine(cv::Mat& org, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) const {
    filter_baseline->setImage(org);
    filter_baseline->doFilter();

    canny->setImage(filter_baseline->getResult());
    canny->doCanny();

    morph->setImage(canny->getResult());
    morph->doMorph();

    hough->setImage(morph->getResult());
    hough->doHorizontalHough();
}

/**
 * \brief Computes the location of the marking rectangle, this rectangle is used to determin the location where the laser is actually on the marking.
 * \param hough The houghline class
 * \return The rectangle which was computed
 */
cv::Rect2d ThicknessGauge::computerMarkingRectangle(shared_ptr<HoughLinesR>& hough) {

    const std::string window_name = "test marking out";

    if (showWindows_)
        draw::makeWindow(window_name);

    filter_marking->setKernel(filters::kernel_line_left_to_right);

    vector<cv::Rect2d> markings(frameCount_);
    vector<cv::Vec4d> left_borders(frameCount_);
    vector<cv::Vec4d> right_borders(frameCount_);

    cv::Rect2d output(0.0, 0.0, 0.0, 0.0);
    cv::Vec4d left_border_result(0.0, 0.0, 0.0, 0.0);
    cv::Vec4d right_border_result(0.0, 0.0, 0.0, 0.0);

    auto running = true;

    auto image_height = static_cast<double>(imageSize_.height);

    unsigned int frame_index = 2;

    auto frames = frameset[frame_index].get();

    log_time << cv::format("computerMarkingRectangle using exposure set %i : %s (%i)\n", frame_index, frames->exp_ext, frames->exp_ms);

    auto accuRects = [image_height](vector<cv::Rect2d>& rects, cv::Rect2d& out) {
        out.x = 0.0;
        out.y = 0.0;
        out.width = 0.0;
        out.height = image_height;
        for (auto& r : rects) {
            out.x += r.x;
            out.width += r.width;
        }
        out.x /= rects.size();
        out.width /= rects.size();
    };

    auto accuVecs = [image_height](vector<cv::Vec4d>& vecs, cv::Vec4d& out) {
        out[0] = 0.0;
        out[1] = image_height;
        out[2] = 0.0;
        out[3] = 0.0;
        for (auto& v : vecs) {
            out[0] += v[0];
            out[1] += v[1];
            out[2] += v[2];
            out[3] += v[3];
        }
        out[0] /= vecs.size();
        out[1] /= vecs.size();
        out[2] /= vecs.size();
        out[3] /= vecs.size();
    };

    while (running) {

        markings.clear();
        left_borders.clear();
        right_borders.clear();

        cv::Mat sparse;
        for (auto i = 0; i < frames->frames.size(); i++) {
            filter_marking->setImage(frames->frames[i].clone());
            filter_marking->doFilter();
            canny->setImage(filter_marking->getResult());
            canny->doCanny();

            auto t = canny->getResult();

            auto tmp = t.clone();
            hough->setOriginal(tmp);
            hough->setImage(t);

            hough->doVerticalHough();
            hough->computeBorders();
            markings.emplace_back(hough->getMarkingRect());
            left_borders.emplace_back(hough->getLeftBorder());
            right_borders.emplace_back(hough->getRightBorder());
            if (draw::is_escape_pressed(30))
                running = false;
        }

        accuRects(markings, output);
        accuVecs(left_borders, left_border_result);
        accuVecs(right_borders, right_border_result);

        if (!showWindows_)
            running = false;
        else {
            auto marking_test = frames->frames.front().clone();
            draw::drawRectangle(marking_test, output, cv::Scalar(128, 128, 128));
            draw::showImage(window_name, marking_test);
            if (draw::is_escape_pressed(30))
                running = false;
        }
    }

    if (showWindows_)
        draw::removeWindow(window_name);

    if (validate::validate_rect(output)) {
        data->leftBorder = left_border_result;
        data->rightBorder = right_border_result;
        hough->leftBorder(left_border_result);
        hough->rightBorder(right_border_result);
        return cv::Rect2d(output);
    }

    return cv::Rect2d(0.0, 0.0, 0.0, 0.0);

}

/**
 * \brief Computes the laser line location on the marking in Y
 * \param laser The laser class
 * \param filter The custom filter class
 */
void ThicknessGauge::computeLaserLocations(shared_ptr<LaserR>& laser, shared_ptr<FilterR>& filter) {

    // generate frames with marking
    std::vector<cv::Mat> marking_frames;

    unsigned int frame_index = 0;

    auto frames = frameset[frame_index].get();

    log_time << cv::format("computeLaserLocations using exposure set %i : %s (%i)\n", frame_index, frames->exp_ext, frames->exp_ms);

    for (auto& frame : frames->frames)
        marking_frames.emplace_back(frame(data->markingRect));

    const std::string window_name = "test height";
    if (showWindows_)
        draw::makeWindow(window_name);

    auto image_size = marking_frames.front().size();

    // local copy of real baseline
    auto base = calc::avg_y(data->baseLines);
    log_time << "baseline vector : " << data->baseLines << endl;

    cv::Mat tmpOut;
    cv::Rect rect_draw;

    auto running = true;

    std::vector<cv::Point2d> results(image_size.width);

    stl::populate_x(results, image_size.width);

    auto failures = 0;

    while (running) {

        auto avg_height = 0.0;

        stl::reset_point_y(results);

        for (auto i = frameCount_; i--;) {

            try {

                cv::Mat base_frame;

                // TODO : replace with custom filter if needed
                cv::bilateralFilter(marking_frames[i], base_frame, 3, 20, 10);

                //cv::Mat t;
                //GenericCV::adaptiveThreshold(baseFrame, t, &thresholdLevel);

                threshold(base_frame, base_frame, binaryThreshold_, 255, CV_THRESH_BINARY);

                GaussianBlur(base_frame, base_frame, cv::Size(5, 5), 0, 10, cv::BORDER_DEFAULT);

                /* RECT CUT METHOD */
                avg_height += calc::weighted_avg(base_frame, data->centerPoints);

                throw_assert(validate::valid_pix_vec(data->centerPoints), "Centerpoints failed validation!!!");

                for (auto& centerpoint : data->centerPoints)
                    results[static_cast<int>(centerpoint.x)].y += centerpoint.y;

                if (draw::is_escape_pressed(30))
                    running = false;

                if (showWindows_ && !i && running) {
                    cv::cvtColor(marking_frames[i], tmpOut, CV_GRAY2BGR);
                    //rect_draw = laser_area;
                }

            } catch (std::exception& e) {
                log_time << e.what() << std::endl;
                failures++;
            }

        }

        log_time << cv::format("Center point data gathering failures : %i\n", failures);

        data->pointsStart[1] = data->centerPoints.front().x + data->markingRect.x;

        for (auto& centerpoint : data->centerPoints) {
            results[static_cast<int>(centerpoint.x)].y /= frameCount_;
            if (centerpoint.x + data->markingRect.x < data->pointsStart[1])
                data->pointsStart[1] = centerpoint.x + data->markingRect.x;
        }

        // since theres some issues with using results vector, this works just as fine.
        data->centerPoints.clear();
        stl::copyVector(results, data->centerPoints);

        auto highest_total = avg_height / static_cast<unsigned int>(frameCount_);

        avg_height = 0.0;

        log_time << cv::format("base: %f\n", base);
        log_time << cv::format("highestPixelTotal: %f\n", highest_total);

        data->difference = base - highest_total;
        log_time << cv::format("diff from baseline: %f\n", data->difference);

        if (!running || !showWindows_)
            break;

        if (showWindows_) {
            draw::drawRectangle(tmpOut, rect_draw, cv::Scalar(255, 0, 0));
            draw::drawHorizontalLine(&tmpOut, cvRound(highest_total), cv::Scalar(0, 255, 0));
            draw::drawHorizontalLine(&tmpOut, cvRound(base), cv::Scalar(0, 0, 255));
            draw::drawText(&tmpOut, cv::format("%f pixels", data->difference), TextDrawPosition::UpperLeft, cv::Scalar(255, 255, 255));
            draw::showImage(window_name, tmpOut);
            if (draw::is_escape_pressed(30))
                running = false;
        } else
            break;

    }

    if (showWindows_)
        draw::removeWindow(window_name);

}

[[deprecated("Not really needed anymore, but still hangs around like a bad fruit")]]
void ThicknessGauge::computerInBetween(shared_ptr<FilterR>& filter, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) {

    // temporary function to compute the middle pieces, just for the lulz, not going to be used in calculations (yet).

    std::vector<cv::Mat> left_middle;
    std::vector<cv::Mat> right_middle;

    // reuseable vectors for all parts
    std::vector<cv::Mat> base;
    std::vector<cv::Mat> mark;

    cv::Rect2d left_base_roi;
    cv::Rect2d right_base_roi;

    cv::Rect2d left_laser_roi;
    cv::Rect2d right_laser_roi;

    // grab low exposure frames to get the baseline part of the image
    auto frame_index = 2;

    auto frame = frameset[frame_index].get();

    auto image_size = frame->frames.front().size();

    auto quarter = image_size.height >> 2;

    log_time << "data->markingRect: " << data->markingRect << endl;
    log_time << "data->leftBorder" << data->leftBorder << endl;
    log_time << "data->rightBorder" << data->rightBorder << endl;

    left_base_roi.x = data->leftBorder[0] + (data->leftBorder[2] - data->leftBorder[0]) / 2;
    left_base_roi.y = image_size.height - quarter;

    left_base_roi.width = data->intersections[0] - data->markingRect.x;
    left_base_roi.height = quarter;

    log_time << "in between: left_base_roi -> " << left_base_roi << endl;

    // TODO : insert check for centerline boundries crossing over border
    left_laser_roi.x = data->intersections[0] - data->intersections[1];
    left_laser_roi.y = 0;

    left_laser_roi.width = data->leftBorder[2] - data->intersections[0];
    left_laser_roi.height = image_size.height;

    log_time << "in between: left_laser_roi -> " << left_laser_roi << endl;

    // grab the left left baseline
    for (auto& f : frame->frames)
        base.emplace_back(f(left_base_roi));

    // switch to lower exposure and grab left right side
    frame_index = 0;
    frame = frameset[frame_index].get();
    for (auto& f : frame->frames)
        mark.emplace_back(f(left_laser_roi));

    cv::imwrite("__left_.png", base.front());
    cv::imwrite("__right_.png", mark.front());

}

/**
 * \brief Computes the minimum houghline lenght for properlistic houghline
 * \tparam minLen The minimim length of the line
 * \param rect The rectangle of the marking location
 * \return the computed value, but not less than minLen
 */
double ThicknessGauge::computeHoughPMinLine(double min_len, cv::Rect2d& rect) {
    auto min_line_len = rect.width / 32.0;

    if (min_line_len < min_len)
        min_line_len = min_len;

    return min_line_len;
}

/**
 * \brief Adds the existing null images to the null_ vector for later substraction
 */
void ThicknessGauge::addNulls() {
    std::vector<cv::String> files;
    cv::String folder = "./nulls/";

    cv::glob(folder, files);

    nulls_.clear();
    nulls_.reserve(files.size());

    for (auto& f : files) {
        log_time << cv::format("loading null file : %s\n", f.c_str());
        nulls_.emplace_back(cv::imread(f, CV_8UC1));
        log_time << nulls_.back().size() << endl;
    }

}

/**
 * \brief Loads a glob from disk and stores them in a vector
 * \param glob_name The name of the glob to load (foldername)
 */
void ThicknessGauge::loadGlob(std::string& glob_name) {

    for (auto i = 0; i < frameset.size(); i++) {

        auto frames = frameset[i].get();

        //cout << frames->exp_ext << endl;

        globGenerator.setPattern(glob_name + expusures_short[i]);
        globGenerator.setRecursive(false);
        globGenerator.generateGlob();

        auto files = globGenerator.getFiles();

        if (files.empty()) {
            CV_Error(cv::Error::StsError, cv::format("No files detected in glob : %s\n", glob_name + expusures_short[i]));
        }

        auto size = static_cast<int>(files.size());

        if (size != frameCount_)
            setFrameCount(size);

        frames->frames.clear();
        frames->frames.reserve(size);

        for (auto& file : files)
            frames->frames.emplace_back(cv::imread(file, CV_8UC1));

        frames->frames.shrink_to_fit();

    }

    setImageSize(frameset.front()->frames.front().size());

}

/**
 * \brief Capture set amount of frames from the capture device with given exposure and stores them in a given vector
 * \param frame_index The frameset index where to store the captured frame
 * \param capture_count The amount of images to be captured
 * \param exposure The exposure for the captured image (not implemented yet!)
 */
void ThicknessGauge::captureFrames(unsigned int frame_index, unsigned int capture_count, unsigned long long int exposure) {
    // TODO : add exposure adjustment for camera before capture!!!!

    cv::Mat t;

    capture->open();

    const std::string window_name = "cap";
    if (showWindows_) {
        draw::showWindows = true;
        draw::makeWindow(window_name);
    }

    log_time << "Starting capture..\n";

    ProgressBar pb(static_cast<unsigned long>(capture_count * frameset.size() * 2), "Capturing..", std::cout);
    pb.SetFrequencyUpdate(10);
    pb.SetStyle("=", " ");
    auto pb_pos = 1;

    for (auto& f : frameset) {
        f->frames.clear();
        f->frames.reserve(capture_count);
        capture->capture(capture_count, f->frames, static_cast<unsigned long>(f->exp_ms));
        //log_time << f << '\n';


        //capture->cap.set(CV_CAP_PROP_EXPOSURE, static_cast<double>(f->exp_ms));
        //cout << "capturing with exposure : " << cap.get(CV_CAP_PROP_EXPOSURE) << endl;
        //for (unsigned int i = 0; i++ < capture_count;) {
        //    pb.Progressed(pb_pos++);
        //capture->cap >> t;
        //f->frames.emplace_back(t);
        //pb.Progressed(pb_pos++);
        //draw::showImage(window_name, t);
        //if (draw::is_escape_pressed(30)) {
        //    // nothing :P
        //}
        //}
    }

    //throw_assert(t.rows == 256, "shiet, husk nu at indstille kameraets roi!");

    //setImageSize(t.size());

    pb.Progressed(100);

    cout << endl;

    log_time << "Capture done.\n";

}

bool ThicknessGauge::saveData(string filename) {
    auto f(filename + ".json");
    cv::FileStorage fs(f, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        cerr << "Error while opening " << f << " for output." << endl;
        return false;
    }

    log_time << cv::format("Saving data..\n");

    cv::Vec3i sizes(static_cast<int>(data->leftPoints.size()), static_cast<int>(data->centerPoints.size()), static_cast<int>(data->rightPoints.size()));

    auto& tmp_mat = frameset.front()->frames.front();

    fs << "Filename" << filename;
    fs << "TimeSaved" << get_time_date();
    fs << "ComputeTime" << frameTime_;
    fs << "Difference" << data->difference;
    fs << "MarkingRectangle" << data->markingRect;
    fs << "Intersections" << data->intersections;
    fs << "IntersectionCuts" << data->intersectionCuts;
    fs << "Baselines" << data->baseLines;
    fs << "CenterLine" << data->centerLine;
    fs << "PointSizes" << sizes;
    fs << "ImageSize" << imageSize_;
    fs << "LeftBasePoints" << data->leftPoints;
    fs << "CenterPoints" << data->centerPoints;
    fs << "RightBasePoints" << data->rightPoints;
    fs << "FirstFrame" << tmp_mat;
    fs.release();

    sorter::sort_pixels_x_ascending(data->leftPoints);
    sorter::sort_pixels_x_ascending(data->centerPoints);
    sorter::sort_pixels_x_ascending(data->rightPoints);

    std::ofstream file_output(filename + ".1.left.intensitet.txt");

    auto writeY = [&](auto p) {
        file_output << p.y - data->difference << '\n';
    };

    // left
    std::for_each(data->leftPoints.begin(), data->leftPoints.end(), writeY);
    file_output.close();

    // center
    file_output.open(filename + ".2.center.intensitet.txt");
    std::for_each(data->centerPoints.begin(), data->centerPoints.end(), writeY);
    file_output.close();

    // right
    file_output.open(filename + ".2.right.intensitet.txt");
    std::for_each(data->rightPoints.begin(), data->rightPoints.end(), writeY);
    file_output.close();

    auto total_width = static_cast<int>(data->leftPoints.size() + data->centerPoints.size() + data->rightPoints.size());

    // generate image for output overview and save it.
    cv::Mat overlay;
    cv::cvtColor(frameset.back()->frames.front(), overlay, CV_GRAY2BGR);
    cv::Mat overview = cv::Mat::zeros(overlay.rows, overlay.cols, frameset[0]->frames.front().type());

#ifdef _MSC_VER
#pragma message("MSC compatible compiler detected -- turning off warning 4309")
#pragma warning( disable : 4309)
#endif
    const char default_intensity = static_cast<char>(210);
    cv::Scalar default_col(0, 0, 250.0);
    cv::Scalar default_bw(200.0, 200.0, 200.0);

    auto paintY = [](cv::Mat& image, std::vector<cv::Point2d>& points, double offset_x, double offset_y = 0.0, cv::Scalar col = cv::Scalar(0.0, 0.0, 255.0)) {
        for (auto& p : points) {
            cv::Point p1(cvRound(p.x + offset_x), cvRound(p.y + offset_y));
            cv::line(image, p1, p1, col);
            //image.at<char>(cvRound(p.y), cvRound(p.x + offset)) = default_intensity;
        }
    };

    // testing height stuff drawing HERE

    draw::showWindows = true;
    const std::string diff_text = cv::format("diff (px): %f", data->difference);
    draw::drawText(&overlay, diff_text, tg::TextDrawPosition::UpperRight, default_col);
    draw::drawText(&overview, diff_text, tg::TextDrawPosition::UpperRight, default_bw);

    // LEFT

    const auto cut_buffer = 40;

    cv::Point2d base_left_p1(data->pointsStart[1] - data->leftPoints.size() - cut_buffer, data->baseLines[1]);
    cv::Point2d base_left_p2(data->pointsStart[1] - cut_buffer, base_left_p1.y);
    draw::drawLine(overlay, base_left_p1, base_left_p2, cv::Scalar(255, 0.0, 0.0));
    draw::drawLine(overview, base_left_p1, base_left_p2, cv::Scalar(255, 0.0, 0.0));
    //paintY(overlay, data->leftPoints, data->pointsStart[1] - data->leftPoints.size(), 0.0, default_col);
    //paintY(overview, data->leftPoints, data->pointsStart[1] - data->leftPoints.size(), 0.0, default_bw);

    // CENTER

    cv::Point2d marking_p1(base_left_p2.x + cut_buffer * 2, base_left_p1.y - data->difference);
    cv::Point2d marking_p2(marking_p1.x + data->centerPoints.size() - cut_buffer * 2, marking_p1.y);
    draw::drawLine(overlay, marking_p1, marking_p2, cv::Scalar(255.0, 0.0, 0.0));
    //draw::drawLine(overview, marking_p1, marking_p2, cv::Scalar(255.0, 0.0, 0.0));
    paintY(overlay, data->centerPoints, marking_p1.x - cut_buffer, -data->difference, default_col);
    paintY(overview, data->centerPoints, marking_p1.x - cut_buffer, -data->difference, default_bw);

    // RIGHT

    cv::Point2d base_right_p1(marking_p2.x + cut_buffer * 2, data->baseLines[3]);
    cv::Point2d base_right_p2(base_right_p1.x + data->rightPoints.size(), base_right_p1.y);
    draw::drawLine(overlay, base_right_p1, base_right_p2, cv::Scalar(255.0, 0.0, 0.0));
    draw::drawLine(overview, base_right_p1, base_right_p2, cv::Scalar(255.0, 0.0, 0.0));
    //paintY(overlay, data->rightPoints, base_right_p1.x, -data->difference, default_col);
    //paintY(overview, data->rightPoints, base_right_p1.x, -data->difference, default_bw);

    cv::imshow("overlay", overlay);
    cv::imshow("overview", overview);

    cv::waitKey(0);

    cv::imwrite("_overlay.png", overlay);
    cv::imwrite("_overview.png", overview);

    return true;
}

void ThicknessGauge::computerGaugeLine(cv::Mat& output) {
    //vi aboveLine;

    //if (miniCalc.getActualPixels(allPixels_, aboveLine, baseLine_[0], output.rows)) {
    //	//cout << "Retrived " << aboveLine.size() << " elements above line.\n";
    //	if (miniCalc.computerCompleteLine(aboveLine, gaugeLine_, lineConfig_)) {
    //		//cout << "Computed line fitting... " << gaugeLine_ << "\n";

    //		gaugeLineSet_ = true;

    //		// sort the elements for quick access to first and last (outer points in line)
    //		sort(aboveLine.begin(), aboveLine.end(), miniCalc.sortX);

    //		avgGaugeHeight_ = gaugeLine_[3];

    //		if (showWindows_) {
    //			line(output, cv::Point2f(static_cast<float>(aboveLine.front().x) + gaugeLine_[0], gaugeLine_[3]), cv::Point2f(static_cast<float>(aboveLine.back().x), gaugeLine_[3]), baseColour_, 2, cv::LINE_AA);

    //			//cout << "Average line height : " << output.rows - avgGaugeHeight_ << " elements.\n";
    //		}
    //	}
    //	else {
    //		gaugeLineSet_ = false;
    //		Util::loge("Failed to generate fitted line.");
    //	}
    //}
    //else {
    //	gaugeLineSet_ = false;
    //	Util::loge("Failed to retrive elements above line.");
    //}

}

[[deprecated("Replaced by Line class, which has an improved interface")]]
bool ThicknessGauge::getSparseY(cv::Mat& image, std::vector<cv::Point>& output) const {

    output.reserve(image.cols);

    std::vector<cv::Point> pix;

    pix.reserve(image.cols);

    findNonZero(image, pix);

    // sort the list in X

    sorter::sort_pixels_x_ascending(pix);

    auto x = pix.front().x;
    auto y = 0;
    auto count = 0;
    auto highest = 0;

    for (auto& p : pix) {
        if (p.x != x) {
            if (count > 0) {
                output.emplace_back(cv::Point(x, y));
                count = 0;
            }
            highest = 0;
        }
        auto intensity = pixel::get_intensity(image, p);
        if (intensity >= highest) {
            highest = p.y;
            x = p.x;
            y = p.y;
        }
        count++;
    }

    return output.empty() ^ true;

}

int ThicknessGauge::getBinaryThreshold() const {
    return binaryThreshold_;
}

void ThicknessGauge::setBinaryThreshold(int binaryThreshold) {
    binaryThreshold_ = binaryThreshold;
}

int ThicknessGauge::getFrameCount() const {
    return frameCount_;
}

void ThicknessGauge::setFrameCount(int frameCount) {
    if (!calc::in_between(frameCount, 1, 999))
        frameCount = 25;

    frameCount_ = frameCount;
}

double ThicknessGauge::getFrameTime() const {
    return frameTime_;
}

bool ThicknessGauge::isSaveVideo() const {
    return saveVideo_;
}

void ThicknessGauge::setSaveVideo(bool saveVideo) {
    saveVideo_ = saveVideo;
}

bool ThicknessGauge::isShowWindows() const {
    return showWindows_;
}

void ThicknessGauge::setShowWindows(bool showWindows) {
    showWindows_ = showWindows;
    draw::showWindows = showWindows;
}
