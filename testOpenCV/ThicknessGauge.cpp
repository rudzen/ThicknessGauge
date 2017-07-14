#include <algorithm>
#include <array>
#include <opencv2/core/base.hpp>

#include "ThicknessGauge.h"

#include "IO/ImageSave.h"
#include "IO/GlobGenerator.h"

#include "UI/ProgressBar.h"

#include "Histogram/Histogram.h"
#include "Histogram/HistoPeak.h"

#include "CV/Data.h"
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
#include "namespaces/cvr.h"
#include "namespaces/draw.h"
#include <future>

using namespace tg;

/**
 * \brief Initializes the class and loads any aditional information
 * \param glob_name if "camera", use camera, otherwise load from glob folder
 */
bool ThicknessGauge::initialize(std::string& glob_name) {
    pdata->glob_name = glob_name;
    //addNulls();

    // determin where to get the frames from.
    if (glob_name == "camera") {

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
            log_err << "Capture device could not be initialized, aborting.\n";
            return false;
        }

        //initVideoCapture(); // for opencv capture object

        capture_device_ok = pcapture->open();

        if (!capture_device_ok) {
            log_err << "Capture device could not be openened, aborting.\n";
            pcapture->initialized(false);
            return false;
        }

        //capture->print_attr();

        pcapture->pixel_format();

        pcapture->reset_binning();

        pcapture->packet_size(8228);

        auto def_roi = cv::Rect_<unsigned long>(0, 1006, 2448, 256);

        pcapture->region(def_roi);

        pcapture->frame_init();

        pcapture->cap_init();

        pcapture->aquisition_init();

        for (auto& fs : frameset_) {
            pcapture->exposure(fs->exp_ms_);
            pcapture->cap(25, fs->frames_);
        }

        pcapture->aquisition_end();

        pcapture->cap_end();

        pcapture->close();

        pcapture->uninitialize();

        //captureFrames(0, frameCount_, 5000);
    } else
        glob_load(glob_name);

    return true;
}

/**
 * \brief Initializes the capture device using PV_API constant
 * (requires that OpenCV is compiled with the location of the PvAPI, deprecated version)
 */
void ThicknessGauge::init_video_capture() {

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
void ThicknessGauge::init_calibration_settings(string fileName) const {
    cs->readSettings(fileName);
}

/**
 * \brief Generates a custom glob
 * \param name The name of the glob, let it be a valid foldername!!!
 */
void ThicknessGauge::glob_generate(std::string& name) {
    init_video_capture();

    file::create_directory(name);
    auto pb_title = "Capturing glob " + name;

    ProgressBar pb(frame_count_, pb_title.c_str());
    pb.set_frequency_update(10);
    pb.set_style("-", " ");

    cv::Mat t;

    cv::VideoCapture cap;
    cap.open(CV_CAP_PVAPI);

    unsigned long progress = 0;
    for (auto i = 0; i < frame_count_; ++i) {
        pb.progressed(++progress);
        cap >> t;
        cv::imwrite(name + "/img" + to_string(i) + ".png", t);
    }
    cap.release();
    pb.progressed(frame_count_);
}

/**
 * \brief Determins the marking boundries
 * \return 2 Float vector with the points marking the boundries as pair, where first = left, second = right
 */
void ThicknessGauge::compute_marking_height() {

    // test print for all framesets
    for (auto& f: frameset_) {
        f->compute();
        //log_time << f << endl;
    }

    while (true) {

        try {

            uint64 time_start = cv::getTickCount();

            // configure frames based on center vertical splitting of the original frames
            //vector<cv::Mat> leftFrames(frameCount_);
            //vector<cv::Mat> rightFrames(frameCount_);
            //splitFrames(leftFrames, rightFrames);

            // configure filters for show window
            pfilter_marking->show_windows(show_windows_);
            pfilter_baseline->show_windows(show_windows_);

            // houghlines to determin where the actual marking is in the frame
            auto hough_vertical = make_shared<HoughLinesR>(1, static_cast<const int>(calc::DEGREES), 40, show_windows_);

            // configure the diffrent functionalities
            hough_vertical->angle_limit(30);
            hough_vertical->show_windows(show_windows_);

            hough_vertical->marking_rect(compute_marking_rectangle(hough_vertical));
            pdata->marking_rect = hough_vertical->marking_rect();

            log_time << "1 ok\n";

            // check the resulting rectangle for weirdness
            //if (data->markingRect.x < 0.0 || data->markingRect.y < 0.0 || data->markingRect.width > imageSize_.width || data->markingRect.height > imageSize_.height || data->markingRect.area() >= imageSize_.area()) {
            //    CV_Error(cv::Error::StsBadSize, cv::format("Marking rectangle has bad size : [x:%f] [y:%f] [w:%f] [h:%f]\n", data->markingRect.x, data->markingRect.y, data->markingRect.width, data->markingRect.height));
            //}

            // make sure the minimum is at least 10 pixels.
            auto min_line_len = calc::line::compute_houghp_min_line(10.0, pdata->marking_rect);

            log_time << "2 ok\n";

            // horizontal houghline extension class
            auto hough_horizontal = make_shared<HoughLinesPR>(1, calc::round(calc::DEGREES), 40, calc::round(min_line_len), show_windows_);

            log_time << "3 ok\n";

            hough_horizontal->max_line_gab(12);
            hough_horizontal->marking_rect(pdata->marking_rect);
            hough_horizontal->show_windows(show_windows_);

            log_time << "4 ok\n";

            // morph extension class
            auto morph = make_shared<MorphR>(cv::MORPH_GRADIENT, 1, show_windows_);

            compute_base_line_areas(hough_horizontal, morph);
            //std::cout << cv::format("Base line Y [left] : %f\n", data->baseLines[1]);
            //std::cout << cv::format("Base line Y [right]: %f\n", data->baseLines[3]);

            // compute the intersection points based on the borders of the markings and the baseline for the laser outside the marking
            calc::compute_intersection_points(pdata->base_lines, hough_vertical->left_border(), hough_vertical->right_border(), pdata->intersections);

            log_time << "5 ok\n";

            // grabs the in between parts and stores the data
            //computerInBetween(filter_baseline, hough_horizontal, morph);

            log_ok << "intersection points: " << pdata->intersections << endl;

            // pixel cut off is based on the border of the marking..
            auto intersect_cutoff = calc::compute_intersection_cut(hough_vertical->left_border(), hough_vertical->right_border());

            log_time << "6 ok\n";

            pdata->intersection_cuts[0] = pdata->intersections[0] - intersect_cutoff[0];
            pdata->intersection_cuts[3] = pdata->intersections[3] + intersect_cutoff[1];

            // set data for in-between areas
            pdata->middle_pieces[0] = pdata->intersection_cuts[0];
            pdata->middle_pieces[1] = pdata->intersection_cuts[3] - pdata->intersection_cuts[0];

            //if (!validate::validate_rect(data->markingRect)) {
            //    CV_Error(cv::Error::BadROISize, "Marking rectangle dimensions are fatal.");
            //}

            // adjust the baselines according to the intersection points. (could perhaps be useful in the future)
            cvr::adjust_marking_rect(pdata->marking_rect, pdata->intersections, intersect_cutoff[0]);

            log_time << "7 ok\n";

            // testing angles between baseline.. should be max 5 degrees
            cv::Point2d line_left(pdata->marking_rect.x, pdata->base_lines[1]);
            cv::Point2d line_right(line_left.x + pdata->marking_rect.width, pdata->base_lines[3]);

            auto angle = calc::angle_between_lines(line_left, line_right);
            log_time << cv::format("Angle between baselines: [r/d] = [%f/%f]\n", angle, calc::rad_to_deg(angle));

            log_time << "8 ok\n";

            //std::cout << cv::format("Adjusted marking rect: [x: %f | y: %f | w: %f | h: %f]\n", data->markingRect.x, data->markingRect.y, data->markingRect.width, data->markingRect.height);
            //std::cout << cv::format("Adjusted base line Y [left] : %f\n", data->baseLines[1]);
            //std::cout << cv::format("Adjusted base line Y [right]: %f\n", data->baseLines[3]);

            // filter for laser detection
            auto filter_laser = make_shared<FilterR>("Laser Filter", show_windows_);
            filter_laser->show_windows(show_windows_);

            // main laser class
            auto laser = make_shared<LaserR>();

            // computes the Y locations of the laserline inside the marking rect
            compute_laser_locations(laser, filter_laser);

            if (show_windows_)
                draw::removeAllWindows();

            // do a quick pass of validation before modifying the data
            validate::valid_data(pdata);

            auto adjust_points = [&](cv::Point2d& p) {
                p.y = image_size_.height - p.y;
            };

            std::for_each(pdata->left_points.begin(), pdata->left_points.end(), adjust_points);
            std::for_each(pdata->right_points.begin(), pdata->right_points.end(), adjust_points);
            std::for_each(pdata->center_points.begin(), pdata->center_points.end(), adjust_points);

            uint64 time_end = cv::getTickCount();

            frame_time_ = static_cast<double>((time_end - time_start) / cv::getTickFrequency());

            log_ok << "Total compute time (seconds) : " << frame_time_ << endl;

            if (show_windows_ && !draw::is_escape_pressed(30))
                continue;

            break;
        } catch (cv::Exception& e) {
            log_err << cv::format("CV Exception caught in computeMarkingHeight().\n%s\n", e.msg.c_str());
        } catch (std::exception& ex) {
            log_err << cv::format("Exception caught in computeMarkingHeight().\n%s\n", ex.what());
        }

    }

}

/**
 * \brief Computes the base line areas and determine the actual base line.
 * \param hough The houghlines class
 * \param morph The morphology class
 */
void ThicknessGauge::compute_base_line_areas(shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) {

    pfilter_baseline->kernel(filters::kernel_line_left_to_right);

    morph->method(cv::MORPH_GRADIENT);
    morph->iterations(1);

    pcanny->threshold_1(200);
    pcanny->threshold_2(250);

    const std::string window_left = "test baseline left";
    const std::string window_right = "test baseline right";

    if (show_windows_) {
        draw::makeWindow(window_left);
        draw::makeWindow(window_right);
    }

    auto quarter = static_cast<double>(image_size_.height) / 4.0;
    auto base_line_y = image_size_.height - quarter;

    auto marking = hough->marking_rect();

    log_time << marking << std::endl;

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
    right_baseline.width = image_size_.width - right_baseline.x;
    right_baseline.height = quarter;

    if (!validate::validate_rect(right_baseline)) {
        log_time << "Validation error for right_baseline in computeBaseLineAreas()." << endl;
    }

    vector<cv::Mat> left_frames;
    vector<cv::Mat> right_frames;

    //frame pointer for the desired frame set
    unsigned int frame_index = 2;

    auto frames = frameset_[frame_index].get();

    log_time << cv::format("computeBaseLineAreas using exposure set %i : %s (%i)\n", frame_index, frames->exp_ext_, frames->exp_ms_);

    // generate baseline images..
    for (auto i = frame_count_; i--;) {
        left_frames.emplace_back(frames->frames_[i](left_baseline));
        right_frames.emplace_back(frames->frames_[i](right_baseline));
    }

    auto left_size = left_frames.front().size();
    auto left_cutoff = left_size.width / 2.0;
    auto right_size = right_frames.front().size();
    auto right_cutoff = right_size.width / 2.0;

    auto left_y = 0.0;
    auto right_y = 0.0;

    std::vector<cv::Point2f> left_elements(left_size.area());
    std::vector<cv::Point2f> right_elements(right_size.area());

    auto offset_y = image_size_.height - quarter;

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

        try {

            // left

            for (auto& left : left_frames) {
                org = left.clone();
                auto h = left.clone();
                hough->original(h);

                process_mat_for_line(org, hough, morph);

                const auto& lines = hough->right_lines(); // inner most side
                for (auto& line : lines)
                    if (line.entry_[0] > left_cutoff)
                        stl::copy_vector(line.elements_, left_elements);

                if (show_windows_ && draw::is_escape_pressed(30))
                    running = false;

            }

            // generate real boundry
            auto left_boundry = cv::minAreaRect(left_elements);
            auto left_boundry_rect = left_boundry.boundingRect();

            log_time << "left_boundry_rect: " << left_boundry_rect.y << endl;

            left_boundry_rect.width -= 40;

            if (show_windows_) {
                draw::drawRectangle(org, left_boundry_rect, cv::Scalar(255, 255, 255));
                draw::showImage(window_left, org);
                if (draw::is_escape_pressed(30))
                    running = false;
            }

            auto t = org(left_boundry_rect);
            left_y = static_cast<double>(left_boundry_rect.y);
            left_y += offset_y;
            left_y += calc::real_intensity_line(t, pdata->left_points, t.rows, 0);

            log_time << "left baseline: " << left_y << endl;

            // right

            for (auto& right : right_frames) {
                org = right.clone();
                auto h1 = right.clone();
                hough->original(h1);

                process_mat_for_line(org, hough, morph);

                const auto& lines = hough->left_lines(); // inner most side
                for (auto& h : lines) {
                    if (h.entry_[2] < right_cutoff)
                        stl::copy_vector(h.elements_, right_elements);
                }

                if (show_windows_ && draw::is_escape_pressed(30))
                    running = false;

            }

            // generate real boundry
            auto right_boundry = cv::minAreaRect(right_elements);
            auto right_boundry_rect = right_boundry.boundingRect();

            right_boundry_rect.x += 40;

            if (show_windows_) {
                draw::drawRectangle(org, right_boundry_rect, cv::Scalar(255, 255, 255));
                draw::showImage(window_right, org);
                if (draw::is_escape_pressed(30))
                    running = false;
            }

            log_time << "right_boundry_rect: " << right_boundry_rect.y << endl;

            t = org(right_boundry_rect);
            right_y = static_cast<double>(right_boundry_rect.y);
            right_y += calc::real_intensity_line(t, pdata->right_points, t.rows, 0);
            right_y += offset_y;

            log_time << "right baseline: " << right_y << endl;

            // forcefully break out of the loop
            if (!show_windows_)
                break;

            //running = false;

        } catch (cv::Exception& e) {
            log_err << __FUNCTION__ << " cv exception caugth " << e.what();
            continue;
        }

    }

    pdata->base_lines[0] = 0.0;
    pdata->base_lines[1] = left_y;
    pdata->base_lines[2] = 0.0;
    pdata->base_lines[3] = right_y;

    if (!validate::valid_vec<double, 4>(pdata->base_lines)) {
        log_time << "Validation error for data->baseLines in computeBaseLineAreas()." << endl;
    }

    if (show_windows_) {
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
void ThicknessGauge::process_mat_for_line(cv::Mat& org, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) const {
    pfilter_baseline->image(org);
    pfilter_baseline->do_filter();

    pcanny->image(pfilter_baseline->result());
    pcanny->do_canny();

    morph->image(pcanny->result());
    morph->morph();

    hough->image(morph->result());
    hough->hough_horizontal();
}

/**
 * \brief Computes the location of the marking rectangle, this rectangle is used to determin the location where the laser is actually on the marking.
 * \param hough The houghline class
 * \return The rectangle which was computed
 */
cv::Rect2d ThicknessGauge::compute_marking_rectangle(shared_ptr<HoughLinesR>& hough) {

    const std::string window_name = "test marking out";

    if (show_windows_)
        draw::makeWindow(window_name);

    pfilter_marking->kernel(filters::kernel_line_right_to_left);

    vector<cv::Rect2d> markings(frame_count_);
    vector<cv::Vec4d> left_borders(frame_count_);
    vector<cv::Vec4d> right_borders(frame_count_);

    cv::Rect2d output(0.0, 0.0, 0.0, 0.0);
    cv::Vec4d left_border_result(0.0, 0.0, 0.0, 0.0);
    cv::Vec4d right_border_result(0.0, 0.0, 0.0, 0.0);

    auto running = true;

    auto image_height = static_cast<double>(image_size_.height);

    unsigned int frame_index = 1;

    auto frames = frameset_[frame_index].get();

    log_time << cv::format("computerMarkingRectangle using exposure set %i : %s (%i)\n", frame_index, frames->exp_ext_, frames->exp_ms_);

    auto accuRects = [image_height](const vector<cv::Rect2d>& rects, cv::Rect2d& out) {
        out.x = 0.0;
        out.y = 0.0;
        out.width = 0.0;
        out.height = image_height;
        for (const auto& r : rects) {
            if (!validate::validate_rect(r))
                continue;
            out.x += r.x;
            out.width += r.width;
        }
        out.x /= rects.size();
        out.width /= rects.size();
    };

    auto accuVecs = [image_height](const vector<cv::Vec4d>& vecs, cv::Vec4d& out) {
        out[0] = 0.0;
        out[1] = image_height;
        out[2] = 0.0;
        out[3] = 0.0;
        for (const auto& v : vecs) {
            if (!validate::valid_vec(v))
                continue;
            out[0] += v[0];
            //out[1] += v[1];
            out[2] += v[2];
            out[3] += v[3];
        }
        out[0] /= vecs.size();
        //out[1] /= vecs.size();
        out[2] /= vecs.size();
        out[3] /= vecs.size();
        //log_time << __FUNCTION__ << " accuVecs 0 : " << out << std::endl;
    };

    while (running) {
        try {

            markings.clear();
            left_borders.clear();
            right_borders.clear();

            cv::Mat sparse;
            for (auto i = 0; i < frames->frames_.size(); i++) {

                //log_time << "frame : " << i << " / " << frames->frames.size() << std::endl;

                //canny->setImage(frames->frames[i].clone());
                //canny->doCanny();

                pfilter_marking->image(frames->frames_[i].clone());
                pfilter_marking->do_filter();
                pcanny->image(pfilter_marking->result());
                pcanny->do_canny();

                auto t = pcanny->result();

                //log_time << "hough\n";

                auto tmp = t.clone();
                hough->original(tmp);
                hough->image(t);

                //log_time << "hough2\n";

                if (hough->hough_vertical() < 0) {
                    log_err << "No lines detected from houghR\n";
                    //continue;
                }

                //log_time << "hough3\n";

                hough->compute_borders();

                auto lb = hough->left_border();
                auto rb = hough->right_border();
                auto mr = hough->marking_rect();

                //markings.emplace_back(mr);
                //left_borders.emplace_back(lb);
                //right_borders.emplace_back(rb);

                if (validate::validate_rect(mr))
                    markings.emplace_back(mr);

                if (validate::valid_vec(lb))
                    left_borders.emplace_back(lb);

                if (validate::valid_vec(rb))
                    right_borders.emplace_back(rb);

                if (show_windows_ && draw::is_escape_pressed(30))
                    running = false;

            }

            accuRects(markings, output);
            accuVecs(left_borders, left_border_result);
            accuVecs(right_borders, right_border_result);

            for (const auto& lb : left_borders) {
                if (!validate::valid_vec(lb)) {
                    log_err << __FUNCTION__ << " left_borders validation fail for " << lb << std::endl;
                }
            }

            for (const auto& lb : right_borders) {
                if (!validate::valid_vec(lb)) {
                    log_err << __FUNCTION__ << " right_borders validation fail for " << lb << std::endl;
                }
            }

            if (!show_windows_)
                running = false;
            else {
                auto marking_test = frames->frames_.front().clone();
                draw::drawRectangle(marking_test, output, cv::Scalar(128, 128, 128));
                draw::showImage(window_name, marking_test);
                if (draw::is_escape_pressed(30))
                    running = false;
            }
        } catch (cv::Exception& e) {
            log_err << "CV Exception\n" << e.what();
            exit(-991);
        } catch (NoLineDetectedException& e) {
            log_err << cv::format("NoLineDetectedException : %s\n", e.what());
            exit(-100);
        }

    }

    if (show_windows_)
        draw::removeWindow(window_name);

    output.height = image_height;

    log_ok << __FUNCTION__ << " : " << output << std::endl;
    //    if (validate::validate_rect(output)) {
    pdata->left_border = left_border_result;
    pdata->right_border = right_border_result;
    hough->left_border(left_border_result);
    hough->right_border(right_border_result);
    return cv::Rect2d(output);
    //    }

    //  return cv::Rect2d(0.0, 0.0, 0.0, 0.0);

}

/**
 * \brief Computes the laser line location on the marking in Y
 * \param laser The laser class
 * \param filter The custom filter class
 */
void ThicknessGauge::compute_laser_locations(shared_ptr<LaserR>& laser, shared_ptr<FilterR>& filter) {

    // generate frames with marking
    std::vector<cv::Mat> marking_frames;

    unsigned int frame_index = 0;

    auto frames = frameset_[frame_index].get();

    log_time << cv::format("computeLaserLocations using exposure set %i : %s (%i)\n", frame_index, frames->exp_ext_, frames->exp_ms_);

    for (auto& frame : frames->frames_)
        marking_frames.emplace_back(frame(pdata->marking_rect));

    const std::string window_name = "test height";
    if (show_windows_)
        draw::makeWindow(window_name);

    auto image_size = marking_frames.front().size();

    // local copy of real baseline
    auto base = calc::avg_y(pdata->base_lines);
    log_time << "baseline vector : " << pdata->base_lines << endl;

    cv::Mat tmpOut;
    cv::Rect rect_draw;

    auto running = true;

    std::vector<cv::Point2d> results(image_size.width);

    stl::populate_x(results, image_size.width);

    auto failures = 0;

    while (running) {

        auto avg_height = 0.0;

        stl::reset_point_y(results);

        cv::Rect laser_y_out;

        for (auto i = frame_count_; i--;) {

            try {

                cv::Mat base_frame;

                // TODO : replace with custom filter if needed
                cv::bilateralFilter(marking_frames[i], base_frame, 3, 20, 10);

                //cv::Mat t;
                //GenericCV::adaptiveThreshold(baseFrame, t, &thresholdLevel);

                threshold(base_frame, base_frame, binary_threshold_, 255, CV_THRESH_BINARY);

                GaussianBlur(base_frame, base_frame, cv::Size(5, 5), 0, 10, cv::BORDER_DEFAULT);

                /* RECT CUT METHOD */
                avg_height += calc::weighted_avg(base_frame, base_frame, pdata->center_points, laser_y_out);
                avg_height += laser_y_out.y;

                throw_assert(validate::valid_pix_vec(pdata->center_points), "Centerpoints failed validation!!!");

                for (auto& centerpoint : pdata->center_points)
                    results[static_cast<int>(centerpoint.x)].y += centerpoint.y;

                if (show_windows_ && draw::is_escape_pressed(30))
                    running = false;

                if (show_windows_ && !i && running) {
                    cv::cvtColor(marking_frames[i], tmpOut, CV_GRAY2BGR);
                    //rect_draw = laser_area;
                }

            } catch (std::exception& e) {
                log_err << e.what() << std::endl;
                failures++;
            }

        }

        log_time << cv::format("Center point data gathering failures : %i\n", failures);

        pdata->points_start[1] = pdata->center_points.front().x + pdata->marking_rect.x;

        for (auto& centerpoint : pdata->center_points) {
            results[static_cast<int>(centerpoint.x)].y /= frame_count_;
            if (centerpoint.x + pdata->marking_rect.x < pdata->points_start[1])
                pdata->points_start[1] = centerpoint.x + pdata->marking_rect.x;
        }

        // since theres some issues with using results vector, this works just as fine.
        pdata->center_points.clear();
        stl::copy_vector(results, pdata->center_points);

        auto highest_total = avg_height / static_cast<unsigned int>(frame_count_);

        avg_height = 0.0;

        log_time << cv::format("base: %f\n", base);
        log_time << cv::format("highestPixelTotal: %f\n", highest_total);

        pdata->difference = base - highest_total;
        log_time << cv::format("diff from baseline: %f\n", pdata->difference);

        if (!running || !show_windows_)
            break;

        if (show_windows_) {
            draw::drawRectangle(tmpOut, rect_draw, cv::Scalar(255, 0, 0));
            draw::drawHorizontalLine(&tmpOut, calc::round(highest_total), cv::Scalar(0, 255, 0));
            draw::drawHorizontalLine(&tmpOut, calc::round(base), cv::Scalar(0, 0, 255));
            draw::drawText(&tmpOut, cv::format("%f pixels", pdata->difference), TextDrawPosition::UpperLeft, cv::Scalar(255, 255, 255));
            draw::showImage(window_name, tmpOut);
            if (draw::is_escape_pressed(30))
                running = false;
        } else
            break;

    }

    if (show_windows_)
        draw::removeWindow(window_name);

}

[[deprecated("Not really needed anymore, but still hangs around like a bad fruit")]]
void ThicknessGauge::computer_in_between(shared_ptr<FilterR>& filter, shared_ptr<HoughLinesPR>& hough, shared_ptr<MorphR>& morph) {

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

    auto frame = frameset_[frame_index].get();

    auto image_size = frame->frames_.front().size();

    auto quarter = image_size.height >> 2;

    log_time << "data->markingRect: " << pdata->marking_rect << endl;
    log_time << "data->leftBorder" << pdata->left_border << endl;
    log_time << "data->rightBorder" << pdata->right_border << endl;

    left_base_roi.x = pdata->left_border[0] + (pdata->left_border[2] - pdata->left_border[0]) / 2;
    left_base_roi.y = image_size.height - quarter;

    left_base_roi.width = pdata->intersections[0] - pdata->marking_rect.x;
    left_base_roi.height = quarter;

    log_time << "in between: left_base_roi -> " << left_base_roi << endl;

    // TODO : insert check for centerline boundries crossing over border
    left_laser_roi.x = pdata->intersections[0] - pdata->intersections[1];
    left_laser_roi.y = 0;

    left_laser_roi.width = pdata->left_border[2] - pdata->intersections[0];
    left_laser_roi.height = image_size.height;

    log_time << "in between: left_laser_roi -> " << left_laser_roi << endl;

    // grab the left left baseline
    for (auto& f : frame->frames_)
        base.emplace_back(f(left_base_roi));

    // switch to lower exposure and grab left right side
    frame_index = 0;
    frame = frameset_[frame_index].get();
    for (auto& f : frame->frames_)
        mark.emplace_back(f(left_laser_roi));

    cv::imwrite("__left_.png", base.front());
    cv::imwrite("__right_.png", mark.front());

}

/**
 * \brief Adds the existing null images to the null_ vector for later substraction
 */
void ThicknessGauge::glob_add_nulls() {
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
void ThicknessGauge::glob_load(std::string& glob_name) {

    GlobGenerator globGenerator;

    // have to read from 0 to size - 1, for 
    for (auto i = 0; i < frameset_.size(); i++) {

        auto frames = frameset_[i].get();

        //cout << frames->exp_ext << endl;

        globGenerator.pattern(glob_name + exposures_short_[i]);
        globGenerator.recursive(false);
        globGenerator.generate_glob();

        auto files = globGenerator.files();

        if (files.empty()) {
            CV_Error(cv::Error::StsError, cv::format("No files detected in glob : %s\n", glob_name + exposures_short_[i]));
        }

        auto size = static_cast<int>(files.size());

        if (size != frame_count_)
            frame_count(size);

        frames->frames_.clear();
        frames->frames_.reserve(size);

        for (auto& file : files)
            frames->frames_.emplace_back(cv::imread(file, CV_8UC1));

        frames->frames_.shrink_to_fit();

    }

    image_size(frameset_.front()->frames_.front().size());

}

/**
 * \brief Capture set amount of frames from the capture device with given exposure and stores them in a given vector
 * \param frame_index The frameset index where to store the captured frame
 * \param capture_count The amount of images to be captured
 * \param exposure The exposure for the captured image (not implemented yet!)
 */
void ThicknessGauge::capture_frames(unsigned int frame_index, unsigned int capture_count, unsigned long long int exposure) {
    // TODO : add exposure adjustment for camera before capture!!!!

    cv::Mat t;

    pcapture->open();

    const std::string window_name = "cap";
    if (show_windows_) {
        draw::makeWindow(window_name);
    }

    log_time << "Starting capture..\n";

    ProgressBar pb(static_cast<unsigned long>(capture_count * frameset_.size() * 2), "Capturing..", std::cout);
    pb.set_frequency_update(10);
    pb.set_style("=", " ");
    auto pb_pos = 1;

    for (auto& f : frameset_) {
        f->frames_.clear();
        f->frames_.reserve(capture_count);
        //capture->cap(capture_count, f->frames, static_cast<unsigned long>(f->exp_ms));
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

    pb.progressed(100);

    std::cout << endl;

    log_time << "Capture done.\n";

}

bool ThicknessGauge::save_data(string filename) {
    auto f(filename + ".json");
    cv::FileStorage fs(f, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        log_err << "Error while opening " << f << " for output." << endl;
        return false;
    }

    log_time << cv::format("Saving data..\n");

    cv::Vec3i sizes(static_cast<int>(pdata->left_points.size()), static_cast<int>(pdata->center_points.size()), static_cast<int>(pdata->right_points.size()));

    auto& tmp_mat = frameset_.front()->frames_.front();

    fs << "Filename" << filename;
    fs << "TimeSaved" << get_time_date();
    fs << "ComputeTime" << frame_time_;
    fs << "Difference" << pdata->difference;
    fs << "MarkingRectangle" << pdata->marking_rect;
    fs << "Intersections" << pdata->intersections;
    fs << "IntersectionCuts" << pdata->intersection_cuts;
    fs << "Baselines" << pdata->base_lines;
    fs << "CenterLine" << pdata->center_line;
    fs << "PointSizes" << sizes;
    fs << "ImageSize" << image_size_;
    fs << "LeftBasePoints" << pdata->left_points;
    fs << "CenterPoints" << pdata->center_points;
    fs << "RightBasePoints" << pdata->right_points;
    fs << "FirstFrame" << tmp_mat;
    fs.release();

    sorter::sort_pixels_x_ascending(pdata->left_points);
    sorter::sort_pixels_x_ascending(pdata->center_points);
    sorter::sort_pixels_x_ascending(pdata->right_points);

    std::ofstream file_output(filename + ".1.left.intensitet.txt");

    auto writeY = [&](auto p) {
        file_output << p.y - pdata->difference << '\n';
    };

    // left
    std::for_each(pdata->left_points.begin(), pdata->left_points.end(), writeY);
    file_output.close();

    // center
    file_output.open(filename + ".2.center.intensitet.txt");
    std::for_each(pdata->center_points.begin(), pdata->center_points.end(), writeY);
    file_output.close();

    // right
    file_output.open(filename + ".2.right.intensitet.txt");
    std::for_each(pdata->right_points.begin(), pdata->right_points.end(), writeY);
    file_output.close();

    auto total_width = static_cast<int>(pdata->left_points.size() + pdata->center_points.size() + pdata->right_points.size());

    // generate image for output overview and save it.
    cv::Mat overlay;
    cv::cvtColor(frameset_.back()->frames_.front(), overlay, CV_GRAY2BGR);
    cv::Mat overview = cv::Mat::zeros(overlay.rows, overlay.cols, frameset_[0]->frames_.front().type());

#ifdef _MSC_VER
#pragma message("MSC compatible compiler detected -- turning off warning 4309")
#pragma warning( disable : 4309)
#endif
    const auto default_intensity = static_cast<char>(210);
    cv::Scalar default_col(0, 0, 250.0);
    cv::Scalar default_bw(200.0, 200.0, 200.0);

    auto paintY = [](cv::Mat& image, std::vector<cv::Point2d>& points, double offset_x, double offset_y = 0.0, cv::Scalar col = cv::Scalar(0.0, 0.0, 255.0)) {
        for (auto& p : points) {
            cv::Point p1(calc::round(p.x + offset_x), calc::round(p.y + offset_y));
            cv::line(image, p1, p1, col);
            //image.at<char>(calc::round(p.y), calc::round(p.x + offset)) = default_intensity;
        }
    };

    // testing height stuff drawing HERE

    const std::string diff_text = cv::format("diff (px): %f", pdata->difference);
    draw::drawText(&overlay, diff_text, tg::TextDrawPosition::UpperRight, default_col);
    draw::drawText(&overview, diff_text, tg::TextDrawPosition::UpperRight, default_bw);

    // LEFT

    const auto cut_buffer = 40;

    cv::Point2d base_left_p1(pdata->points_start[1] - pdata->left_points.size() - cut_buffer, pdata->base_lines[1]);
    cv::Point2d base_left_p2(pdata->points_start[1] - cut_buffer, base_left_p1.y);
    draw::drawLine(overlay, base_left_p1, base_left_p2, cv::Scalar(255, 0.0, 0.0));
    draw::drawLine(overview, base_left_p1, base_left_p2, cv::Scalar(255, 0.0, 0.0));
    //paintY(overlay, data->leftPoints, data->pointsStart[1] - data->leftPoints.size(), 0.0, default_col);
    //paintY(overview, data->leftPoints, data->pointsStart[1] - data->leftPoints.size(), 0.0, default_bw);

    // CENTER

    cv::Point2d marking_p1(base_left_p2.x + cut_buffer * 2, base_left_p1.y - pdata->difference);
    cv::Point2d marking_p2(marking_p1.x + pdata->center_points.size() - cut_buffer * 2, marking_p1.y);
    draw::drawLine(overlay, marking_p1, marking_p2, cv::Scalar(255.0, 0.0, 0.0));
    //draw::drawLine(overview, marking_p1, marking_p2, cv::Scalar(255.0, 0.0, 0.0));
    paintY(overlay, pdata->center_points, marking_p1.x - cut_buffer, -pdata->difference, default_col);
    paintY(overview, pdata->center_points, marking_p1.x - cut_buffer, -pdata->difference, default_bw);

    // RIGHT

    cv::Point2d base_right_p1(marking_p2.x + cut_buffer * 2, pdata->base_lines[3]);
    cv::Point2d base_right_p2(base_right_p1.x + pdata->right_points.size(), base_right_p1.y);
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

void ThicknessGauge::test_edge() {

    cv::VideoCapture cap(CV_CAP_PVAPI); // open the default camera
    if (!cap.isOpened()) // check if we succeeded
        return;

    cv::namedWindow("Video", 1);
    while (true) {
        cv::Mat frame;
        cap >> frame; // get a new frame from camera
        imshow("Video", frame);

        // Press 'c' to escape
        if (cv::waitKey(30) == 'c')
            break;
    }
    return;
}

void ThicknessGauge::computer_gauge_line(cv::Mat& output) {
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
bool ThicknessGauge::sparse_y(cv::Mat& image, std::vector<cv::Point>& output) const {

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

int ThicknessGauge::binary_threshold() const {
    return binary_threshold_;
}

void ThicknessGauge::binary_threshold(int binaryThreshold) {
    binary_threshold_ = binaryThreshold;
}

int ThicknessGauge::frame_count() const {
    return frame_count_;
}

void ThicknessGauge::frame_count(int frameCount) {
    if (!calc::in_between(frameCount, 1, 999))
        frameCount = 25;

    frame_count_ = frameCount;
}

double ThicknessGauge::frame_time() const {
    return frame_time_;
}

bool ThicknessGauge::save_video() const {
    return save_video_;
}

void ThicknessGauge::save_video(bool saveVideo) {
    save_video_ = saveVideo;
}

bool ThicknessGauge::show_windows() const {
    return show_windows_;
}

void ThicknessGauge::show_windows(bool showWindows) {
    show_windows_ = showWindows;
}
