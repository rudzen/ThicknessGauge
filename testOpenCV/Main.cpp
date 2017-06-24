#include <opencv2/opencv.hpp>
#include <iostream>
#include <VimbaCPP/Include/VimbaSystem.h>

#include "tclap/CmdLine.h"

#include "ArgClasses/CommandLineOptions.h"
#include "ArgClasses/IntegerConstraint.h"
#include "ArgClasses/FileConstraint.h"
#include "ArgClasses/DemoModeVisitor.h"
#include "ArgClasses/CalibrationModeVisitor.h"
#include "ArgClasses/TestSuitConstraint.h"
#include "ArgClasses/TestModeVisitor.h"
#include "ArgClasses/BuildInfoVisitor.h"
#include "ArgClasses/GlobModeVisitor.h"

#include "Exceptions/CaptureFailException.h"
#include "Exceptions/CalibrationException.h"
#include "Exceptions/TestException.h"

#include "Calibrate/CalibrationTest.cpp"

#include "ThicknessGauge.h"

#include "namespaces/tg.h"
#include <PvApi.h>
#include <thread>
#include "Camera/CapturePvApi.h"
#include "Camera/Calib.h"

using namespace std;
using namespace TCLAP;
using namespace tg;

/*
 * Application return codes :
 *
 * -1	= Argument parsing error
 * -2	= Capture fail
 * -3	= Calibration error
 * -4	= Test mode exception
 */

#define _USE_MATH_DEFINES

const string default_camera_calibration_file = "C2450.json";

/**
 * \brief Parses the command line arguments and dissolves them into options
 * \param argc Mirrored from main()
 * \param argv Mirrored from main()
 * \param options The options class (DTO)
 * \return true if an action which doesn't allow the thicknessgauge to proceed afterwards, fx show information etc.
 * otherwise false.
 */
bool parse_args(int argc, char** argv, CommandLineOptions& options);

void save_null(std::string filename) {

    // quick and dirty hack to save null image quickly
    log_time << cv::format("Enter delay in seconds before capture to %s\n>", filename);
    int t;
    cin >> t;
    cv::setNumThreads(2);
    cv::Mat nullImage;
    cv::VideoCapture cap;
    cap.open(CV_CAP_PVAPI);
    cap >> nullImage;
    cv::imwrite(filename, nullImage);
    cap.release();
}

int main(int argc, char** argv) {

    // jump directly into vimba testing for now!
    //return testCPP(argc, argv);

    auto return_value = false;
    CommandLineOptions options;

    try {

        if (parse_args(argc, argv, options)) {
            // unique case for build information
            if (options.build_info_mode()) {
                log_time << cv::getBuildInformation();
                return 0;
            }
            // null save mode..
            save_null(options.camera_file());
            return 0;
        }

        auto thickness_gauge = std::make_unique<ThicknessGauge>(options.frames(), options.show_windows(), options.record_video(), 100, 100);

        //thicknessGauge->setFrameCount(options.getFrames());
        //thicknessGauge->setShowWindows(options.isShowWindows());
        //thicknessGauge->setSaveVideo(options.isRecordVideo());
        thickness_gauge->init_calibration_settings(options.camera_file());
        cv::setNumThreads(options.num_open_cv_threads());
        log_time << iif(cv::useOptimized(), "OpenCV is using optimization.", "Warning, OpenCV has optimization disabled.") << '\n';

        if (options.glob_mode()) {
            // TODO : use capture for file reading?!
            thickness_gauge->init_video_capture();
            auto glob_name = options.glob_folder();
            thickness_gauge->glob_generate(glob_name);
        } else if (options.demo_mode()) {// && !options.TestMode() && !options.CalibrationMode()) {

            auto glob_name = options.glob_folder();

            thickness_gauge->glob_add_nulls();

            while (true) {
                auto initialized_ok = thickness_gauge->initialize(glob_name);
                if (initialized_ok)
                    break;

                log_time << "Unable to initialize....\n";
                log_time << "Retrying in a moment [press ctrl-c to abort].\n";
                tg::sleep(500);
            }

            //    log_time << "Catastrofic failure.. exiting..\n";
            //    return -20;
            //}

            thickness_gauge->compute_marking_height();

            auto data = thickness_gauge->pdata; // virker :-)

            thickness_gauge->save_data("output_mufmuf");

            log_time << cv::format("difference: %f\n", data->difference);

            log_time << "done..\n";
        } else if (options.calibration_mode()) {
            Calib calib;
            calib.run_calib();
        } else if (options.test_mode()) {
            //c.initVideoCapture();
            //c.testAggressive();
        }
    } catch (ArgException& ae) {
        string what = ae.what();
        log_timedate << "ArgException suddenly happend (but what?)\n" + what << std::endl;
        return -1;
    }
    catch (CaptureFailException& cfe) {
        string what = cfe.what();
        log_timedate << "CaptureFailException suddenly happend (but what?)\n" + what << std::endl;
        return -2;
    }
    catch (CalibrationException& cale) {
        string what = cale.what();
        log_timedate << "CalibrationException suddenly happend (but what?)\n" + what << std::endl;
        return -3;
    }
    catch (TestException& te) {
        string what = te.what();
        log_timedate << "TestException suddenly happend (but what?)\n" + what << std::endl;
        return -4;
    } catch (cv::Exception& e) {
        cerr << cv::format("cv::Exception caught in main\n", e.msg.c_str());
    }

    log_time << cv::format("Smooth operator >> %i\n", return_value);

    return return_value;

}

bool parse_args(int argc, char** argv, CommandLineOptions& options) {
    try {
        CmdLine cmd("ThicknessGauge [OpenCV]", '=', "0.1", true);

        /* begin singular switches, only one permitted, default is demo mode */

        // xor args
        vector<Arg*> xors;

        // add basic switches
        SwitchArg switch_demo("d", "demo", "runs regular demo", true, new DemoModeVisitor());
        xors.emplace_back(&switch_demo);
        //cmd.add(demoSwitch);

        SwitchArg switch_calibration("c", "calibrate", "perform camera calibration", false, new CalibrationModeVisitor());
        xors.emplace_back(&switch_calibration);
        //cmd.add(calibrationSwitch);

        SwitchArg switch_buildinfo("i", "info", "show software information", false, new BuildInfoVisitor());
        xors.emplace_back(&switch_buildinfo);
        //cmd.add(buildInfoSwitch);

        SwitchArg switch_test("t", "test", "Perform test", false, new TestModeVisitor());
        xors.emplace_back(&switch_test);

        SwitchArg switch_glob("g", "save_glob", "Save -f frames as glob", false, new GlobModeVisitor());
        xors.emplace_back(&switch_glob);

        cmd.xorAdd(xors);

        ValueArg<string> arg_nullsave("", "null_save", "Save a singular image and exit", false, "null.png", new TestSuitConstraint());
        cmd.add(arg_nullsave);

        /* end switches */

        /* begin value base argument*/
        ValueArg<string> arg_testsuite("", "test_suite", "Test name for saving the test under.", false, "default", new TestSuitConstraint());
        cmd.add(arg_testsuite);

        ValueArg<int> arg_frame("", "frames", "amount of frames each calculation", false, 25, new IntegerConstraint("Frames", 5, 200));
        cmd.add(arg_frame);

        ValueArg<bool> arg_show_windows("", "show_windows", "displays windows in demo mode", false, true, "0/1");
        cmd.add(arg_show_windows);

        ValueArg<bool> arg_record_video("", "record_video", "Records demo mode to video", false, false, "0/1");
        cmd.add(arg_record_video);

        ValueArg<string> arg_camera_calibration_file("", "camera_settings", "OpenCV camera calibration file", false, default_camera_calibration_file, new FileConstraint());
        cmd.add(arg_camera_calibration_file);

        ValueArg<string> arg_calibration_output("", "calibrate_output", "output file for camera matricies", false, "output.json", "filename");
        cmd.add(arg_calibration_output);

        ValueArg<int> arg_max_opencv_threads("", "opencv_threads", "OpenCV thread limit", false, 4, new IntegerConstraint("OpenCV Threads", 1, 40));
        cmd.add(arg_max_opencv_threads);

        ValueArg<string> arg_glob_name("", "glob_name", "Name to save glob as", false, "camera", "Valid folder name.");
        cmd.add(arg_glob_name);

        cmd.parse(argc, argv);

        // check for null save.. this is an important thing! :-)
        if (arg_nullsave.isSet()) {
            options.camera_file(arg_nullsave.getValue());
            return true;
        }

        // read all parsed command line arguments
        if (switch_demo.isSet())
            options.demo_mode(true);
        else if (switch_calibration.isSet())
            options.calibration_mode(true);
        else if (switch_test.isSet())
            options.test_mode(true);
        else if (switch_buildinfo.isSet()) { // check, its a instant abort if build info is found
            options.build_info_mode(true);
            return true;
        } else if (switch_glob.isSet())
            options.glob_mode(true); // glob mode

        auto sval = arg_camera_calibration_file.getValue();
        options.camera_file(sval);

        sval = arg_calibration_output.getValue();
        options.calibration_output(sval);

        sval = arg_testsuite.getValue();
        options.test_suite(sval);

        sval = arg_glob_name.getValue();
        options.glob_folder(sval);

        auto ival = arg_frame.getValue();
        options.frames(ival);

        ival = arg_max_opencv_threads.getValue();
        options.num_open_cv_threads(ival);

        auto bval = arg_show_windows.getValue();
        options.show_windows(bval);

        bval = arg_record_video.getValue();
        options.record_video(bval);

        return false;
    } catch (ArgException& e) {
        throw(e);
    }
}
