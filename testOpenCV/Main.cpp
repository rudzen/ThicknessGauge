#include <opencv2/opencv.hpp>
#include <iostream>

#include "tclap/CmdLine.h"

#include "ArgClasses/CommandLineOptions.h"

#include "Exceptions/CaptureFailException.h"
#include "Exceptions/CalibrationException.h"
#include "Exceptions/TestException.h"

#include "Calibrate/CalibrationTest.cpp"

#include "ThicknessGauge.h"

#include "namespaces/tg.h"
#include "Camera/Calib.h"
#include "ArgClasses/args.h"
#include "Camera/Seeker.h"

using namespace tg;

/*
 * Application return codes :
 *
 * -1	= Argument parsing error
 * -2	= Capture fail
 * -3	= Calibration error
 * -4	= Test mode exception
 */


int main(int argc, char** argv) {

    // jump directly into vimba testing for now!
    //return testCPP(argc, argv);

    auto return_value = false;

    auto options = std::make_shared<CommandLineOptions>();

    try {

        if (args::parse_args(argc, argv, options)) {
            // unique case for build information
            if (options->build_info_mode()) {
                log_time << cv::format(cv::getBuildInformation().c_str());
                return 0;
            }
        }

        auto thickness_gauge = std::make_unique<ThicknessGauge>(options->frames(), options->show_windows(), options->record_video(), 100, 100);

        //thicknessGauge->setFrameCount(options.getFrames());
        //thicknessGauge->setShowWindows(options.isShowWindows());
        //thicknessGauge->setSaveVideo(options.isRecordVideo());
        thickness_gauge->init_calibration_settings(options->camera_file());
        cv::setNumThreads(options->num_open_cv_threads());

        if (options->glob_mode()) {
            // TODO : use capture for file reading?!
            thickness_gauge->init_video_capture();
            auto glob_name = options->glob_folder();
            thickness_gauge->glob_generate(glob_name);
        } else if (options->demo_mode()) {// && !options->TestMode() && !options->CalibrationMode()) {

            auto glob_name = options->glob_folder();

            auto seeker = std::make_shared<Seeker>();

            /* **********************************************************
             * To measure zero height, perform a regular height measure,
             * remove the thing that was measured, then input the resulting
             * marking rectangle in the zero_measure_mr rectangle.
             * ALWAYS use the ceiling values, as they are more likely to be more "correct".
             * 
             * The current version of this software leaves the camera ROI where it last set it.
             * Open up the camera software to check the current ROI, this should be the correct one.
             */

            //1173.33 x 256 from (695, 0)
            //e: 44000

            // set to true if doing null-measurements
            auto do_zero = false;

            // fx: [237 x 256 from (1168.25, 0) - 1 keramik
            //cv::Rect_<unsigned long> zero_measure_mr(1161UL, 0UL, 234UL, 256UL);

            // 4 vertikale keramik
            cv::Rect_<unsigned long> zero_measure_mr(800UL, 0UL, 958UL, 256UL);

            // must be defined from "recent" search as it could easily change!
            unsigned long phase_two_exposure = 10000;

            //// determin camera or file storage
            if (glob_name == "camera") {

                while (true) {
                    try {
                        bool ok;

                        if (!do_zero)
                            ok = seeker->compute(false , zero_measure_mr, phase_two_exposure); // default
                        else
                            ok = seeker->compute(true , zero_measure_mr, phase_two_exposure); // force feed the bastard

                        if (ok)
                            break;

                        log_err << cv::format("Unable to initialize....\n");
                        log_err << cv::format("Retrying in a moment [press ctrl-c to abort].\n");
                        tg::sleep(500);
                    } catch (cv::Exception& e) {
                        log_err << cv::format("cv::Exception caught in main\n", e.msg.c_str());
                    }
                }

            } else {

                thickness_gauge->glob_add_nulls();

                while (true) {
                    auto initialized_ok = thickness_gauge->initialize(glob_name);
                    if (initialized_ok)
                        break;

                    log_err << cv::format("Unable to initialize....\n");
                    log_err << cv::format("Retrying in a moment [press ctrl-c to abort].\n");
                    tg::sleep(500);
                }

                thickness_gauge->compute_marking_height();

                auto data = thickness_gauge->pdata; // virker :-)

                thickness_gauge->save_data("output_mufmuf");

                log_ok << cv::format("difference: %f\n", data->difference);
                log_err << cv::format("done..\n");

            }


        } else if (options->calibration_mode()) {
            Calib calib;
            calib.run_calib();
        } else if (options->test_mode()) {
            //c.initVideoCapture();
            //c.testAggressive();
        }
    } catch
    (TCLAP::ArgException& ae) {
        string what = ae.what();
        log_err << "ArgException suddenly happend (but what?)\n" + what << std::endl;
        return -1;
    }
    catch
    (CaptureFailException& cfe) {
        string what = cfe.what();
        log_err << "CaptureFailException suddenly happend (but what?)\n" + what << std::endl;
        return -2;
    }
    catch
    (CalibrationException& cale) {
        string what = cale.what();
        log_err << "CalibrationException suddenly happend (but what?)\n" + what << std::endl;
        return -3;
    }
    catch
    (TestException& te) {
        string what = te.what();
        log_err << "TestException suddenly happend (but what?)\n" + what << std::endl;
        return -4;
    }

    log_time << cv::format("Smooth operator >> %i\n", return_value);

    return return_value;

}
