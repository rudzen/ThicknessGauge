#include "args.h"
#include "ArgClasses/CommandLineOptions.h"
#include <tclap/ArgException.h>
#include <tclap/CmdLine.h>
#include "ArgClasses/DemoModeVisitor.h"
#include "ArgClasses/CalibrationModeVisitor.h"
#include "ArgClasses/BuildInfoVisitor.h"
#include "ArgClasses/TestModeVisitor.h"
#include "ArgClasses/GlobModeVisitor.h"
#include "ArgClasses/TestSuitConstraint.h"
#include "ArgClasses/IntegerConstraint.h"
#include "ArgClasses/FileConstraint.h"

namespace args {
    
    bool parse_args(int argc, char** argv, std::shared_ptr<CommandLineOptions> options) {
    try {
        TCLAP::CmdLine cmd("ThicknessGauge [OpenCV]", '=', "0.1", true);

        /* begin singular switches, only one permitted, default is demo mode */

        // xor args
        std::vector<TCLAP::Arg*> xors;

        // add basic switches
        TCLAP::SwitchArg switch_demo("d", "demo", "runs regular demo", true, new DemoModeVisitor());
        xors.emplace_back(&switch_demo);
        //cmd.add(demoSwitch);

        TCLAP::SwitchArg switch_calibration("c", "calibrate", "perform camera calibration", false, new CalibrationModeVisitor());
        xors.emplace_back(&switch_calibration);
        //cmd.add(calibrationSwitch);

        TCLAP::SwitchArg switch_buildinfo("i", "info", "show software information", false, new BuildInfoVisitor());
        xors.emplace_back(&switch_buildinfo);
        //cmd.add(buildInfoSwitch);

        TCLAP::SwitchArg switch_test("t", "test", "Perform test", false, new TestModeVisitor());
        xors.emplace_back(&switch_test);

        TCLAP::SwitchArg switch_glob("g", "save_glob", "Save -f frames as glob", false, new GlobModeVisitor());
        xors.emplace_back(&switch_glob);

        cmd.xorAdd(xors);

        /* end switches */

        /* begin value base argument*/
        TCLAP::ValueArg<std::string> arg_testsuite("", "test_suite", "Test name for saving the test under.", false, "default", new TestSuitConstraint());
        cmd.add(arg_testsuite);

        TCLAP::ValueArg<int> arg_frame("", "frames", "amount of frames each calculation", false, 25, new IntegerConstraint("Frames", 5, 200));
        cmd.add(arg_frame);

        TCLAP::ValueArg<bool> arg_show_windows("", "show_windows", "displays windows in demo mode", false, true, "0/1");
        cmd.add(arg_show_windows);

        TCLAP::ValueArg<bool> arg_record_video("", "record_video", "Records demo mode to video", false, false, "0/1");
        cmd.add(arg_record_video);

        TCLAP::ValueArg<std::string> arg_camera_calibration_file("", "camera_settings", "OpenCV camera calibration file", false, default_camera_calibration_file, new FileConstraint());
        cmd.add(arg_camera_calibration_file);

        TCLAP::ValueArg<std::string> arg_calibration_output("", "calibrate_output", "output file for camera matricies", false, "output.json", "filename");
        cmd.add(arg_calibration_output);

        TCLAP::ValueArg<int> arg_max_opencv_threads("", "opencv_threads", "OpenCV thread limit", false, 4, new IntegerConstraint("OpenCV Threads", 1, 40));
        cmd.add(arg_max_opencv_threads);

        TCLAP::ValueArg<std::string> arg_glob_name("", "glob_name", "Name to save glob as", false, "camera", "Valid folder name.");
        cmd.add(arg_glob_name);

        cmd.parse(argc, argv);

        // read all parsed command line arguments
        if (switch_demo.isSet())
            options->demo_mode(true);
        else if (switch_calibration.isSet())
            options->calibration_mode(true);
        else if (switch_test.isSet())
            options->test_mode(true);
        else if (switch_buildinfo.isSet()) { // check, its a instant abort if build info is found
            options->build_info_mode(true);
            return true;
        } else if (switch_glob.isSet())
            options->glob_mode(true); // glob mode

        auto sval = arg_camera_calibration_file.getValue();
        options->camera_file(sval);

        sval = arg_calibration_output.getValue();
        options->calibration_output(sval);

        sval = arg_testsuite.getValue();
        options->test_suite(sval);

        sval = arg_glob_name.getValue();
        options->glob_folder(sval);

        auto ival = arg_frame.getValue();
        options->frames(ival);

        ival = arg_max_opencv_threads.getValue();
        options->num_open_cv_threads(ival);

        auto bval = arg_show_windows.getValue();
        options->show_windows(bval);

        bval = arg_record_video.getValue();
        options->record_video(bval);

        return false;
    } catch (TCLAP::ArgException& e) {
        throw(e);
    }
}

}
