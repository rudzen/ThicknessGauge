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

#include "Vimba/GC2450MCamera.h"
#include "Vimba/CameraData.h"
#include "Vimba/CameraFrame.h"

#include "Calibrate/CalibrationTest.cpp"

#include "ThicknessGauge.h"

#include "namespaces/tg.h"
#include <PvApi.h>
#include <thread>
#include "Camera/CapturePvApi.h"

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


#define VIMBA

const string default_camera_calibration_file = "C2450.json";

bool parseArgs(int argc, char** argv, CommandLineOptions& options);

//pvapi testing declarations
int testCPP(int argc, char* argv[]);

void saveNull(std::string filename) {

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

    CapturePvApi capture_;

    capture_.initialize();

    capture_.open();

    std::vector<cv::Mat> temp;

    capture_.capture(25, temp, 60000);

    capture_.close();

    log_time << "frames captured : " << temp.size() << std::endl;

    while (true) {
        int key = 0;
        for (const auto& i : temp) {
            cv::imshow("View window", i);
            key = cv::waitKey(5);
        }
        if (key == 27) {
            std::cout << "Closed down the application by pressing ESC." << std::endl;
            break;
        }
    }

    return 0;

    // jump directly into vimba testing for now!
    //return testCPP(argc, argv);

    auto return_value = false;
    CommandLineOptions options;

    try {

        if (parseArgs(argc, argv, options)) {
            // unique case for build information
            if (options.isBuildInfoMode()) {
                log_time << cv::getBuildInformation();
                return 0;
            }
            // null save mode..
            saveNull(options.getCameraFile());
            return 0;
        }

        auto thicknessGauge = std::make_unique<ThicknessGauge>(options.getFrames(), options.isShowWindows(), options.isRecordVideo(), 100, 100);

        //thicknessGauge->setFrameCount(options.getFrames());
        //thicknessGauge->setShowWindows(options.isShowWindows());
        //thicknessGauge->setSaveVideo(options.isRecordVideo());
        thicknessGauge->initCalibrationSettings(options.getCameraFile());
        cv::setNumThreads(options.getNumOpenCvThreads());

        log_time << cv::format("OpenCV Optimization use : %i\n", cv::useOptimized());

        //log_time << options << endl;

        if (options.isGlobMode()) {
            // TODO : use capture for file reading?!
            thicknessGauge->initVideoCapture();
            auto globName = options.getGlobFolder();
            thicknessGauge->generateGlob(globName);
        } else if (options.isDemoMode()) {// && !options.TestMode() && !options.CalibrationMode()) {

            auto glob_name = options.getGlobFolder();

            thicknessGauge->initialize(glob_name);

            thicknessGauge->computeMarkingHeight();

            auto data = thicknessGauge->getData<double>(); // virker :-)

            thicknessGauge->saveData("output_mufmuf");

            log_time << cv::format("difference: %f\n", data->difference);

            log_time << "done..\n";
        } else if (options.isCalibrationMode()) {
            throw CalibrationException("Unable to initiate calibration mode, feature not completed.");
        } else if (options.isTestMode()) {
            //c.initVideoCapture();
            //c.testAggressive();
        }
    } catch (ArgException& ae) {
        string what = ae.what();
        log_timedate << "Exception suddenly happend (but what?)\n" + what << std::endl;
        return -1;
    }
    catch (CaptureFailException& cfe) {
        string what = cfe.what();
        log_timedate << "Exception suddenly happend (but what?)\n" + what << std::endl;
        return -2;
    }
    catch (CalibrationException& cale) {
        string what = cale.what();
        log_timedate << "Exception suddenly happend (but what?)\n" + what << std::endl;
        return -3;
    }
    catch (TestException& te) {
        string what = te.what();
        log_timedate << "Exception suddenly happend (but what?)\n" + what << std::endl;
        return -4;
    }
    log_time << cv::format("Smooth operator >> %i\n", return_value);

    return 0;

}

bool parseArgs(int argc, char** argv, CommandLineOptions& options) {
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
            options.setCameraFile(arg_nullsave.getValue());
            return true;
        }

        // read all parsed command line arguments
        if (switch_demo.isSet())
            options.setDemoMode(true);
        else if (switch_calibration.isSet())
            options.setCalibrationMode(true);
        else if (switch_test.isSet())
            options.setTestMode(true);
        else if (switch_buildinfo.isSet()) { // check, its a instant abort if build info is found
            options.setBuildInfoMode(true);
            return true;
        } else if (switch_glob.isSet())
            options.setGlobMode(true); // glob mode

        auto sval = arg_camera_calibration_file.getValue();
        options.setCameraFile(sval);

        sval = arg_calibration_output.getValue();
        options.setCalibrationOutput(sval);

        sval = arg_testsuite.getValue();
        options.setTestSuite(sval);

        sval = arg_glob_name.getValue();
        options.setGlobFolder(sval);

        auto ival = arg_frame.getValue();
        options.setFrames(ival);

        ival = arg_max_opencv_threads.getValue();
        options.setNumOpenCvThreads(ival);

        auto bval = arg_show_windows.getValue();
        options.setShowWindows(bval);

        bval = arg_record_video.getValue();
        options.setRecordVideo(bval);

        return false;
    } catch (ArgException& e) {
        throw(e);
    }
}


//using namespace std;
//using namespace cv;
//
//// camera's data type definition
//typedef struct {
//    unsigned long UID;
//    tPvHandle Handle;
//    tPvFrame Frame;
//
//} tCamera;
//
//#define CH_MONO 1   // Single channel for mono images
//
//int testCPP(int argc, char* argv[]) {
//    tg::tCamera myCamera;
//    tPvCameraInfo cameraInfo;
//    unsigned long frameSize;
//    tPvErr Errcode;
//
//    int counter = 0;
//
//    if (argc == 1) {
//        cout << "This script will stream data from an AVT MANTA GigE camera using OpenCV C++ style interface." << endl;
//        cout << "capture_manta.exe <resolution width> <resolution heigth>" << endl;
//        return 0;
//    }
//
//    // Initialize the API
//    if (!PvInitialize()) {
//        // Wait for the response from a camera after the initialization of the driver
//        // This is done by checking if camera's are found yet
//        ////////////////////////////////////////////////////////////
//        while (PvCameraCount() == 0) {
//            //std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        }
//
//        //debug
//        cout << "Camera count successfully finished." << endl;
//
//        /////////////////////////////////////////////////////////////
//        if (PvCameraList(&cameraInfo, 1, NULL) == 1) {
//            // Get the camera ID
//            myCamera.UID = cameraInfo.UniqueId;
//
//            // Open the camera
//            if (!PvCameraOpen(myCamera.UID, ePvAccessMaster, &(myCamera.Handle))) {
//                //debug
//                cout << "Camera opened succesfully." << endl;
//
//                // Get the image size of every capture
//                PvAttrUint32Get(myCamera.Handle, "TotalBytesPerFrame", &frameSize);
//
//                // Allocate a buffer to store the image
//                memset(&myCamera.Frame, 0, sizeof(tPvFrame));
//                myCamera.Frame.ImageBufferSize = frameSize;
//                myCamera.Frame.ImageBuffer = new char[frameSize];
//
//                // Set maximum camera parameters - camera specific
//                int max_width = 1624;
//                int max_heigth = 1234;
//
//                int center_x = max_width / 2;
//                int center_y = max_heigth / 2;
//
//                int frame_width = atoi(argv[1]);
//                int frame_heigth = atoi(argv[2]);
//
//                // Set the manta camera parameters to get wanted frame size retrieved
//                PvAttrUint32Set(myCamera.Handle, "RegionX", center_x - (frame_width / 2));
//                PvAttrUint32Set(myCamera.Handle, "RegionY", center_y - (frame_heigth / 2));
//                PvAttrUint32Set(myCamera.Handle, "Width", frame_width);
//                PvAttrUint32Set(myCamera.Handle, "Height", frame_heigth);
//
//                // Start the camera
//                PvCaptureStart(myCamera.Handle);
//
//                // Set the camera to capture continuously
//                PvAttrEnumSet(myCamera.Handle, "AcquisitionMode", "Continuous");
//                PvCommandRun(myCamera.Handle, "AcquisitionStart");
//                PvAttrEnumSet(myCamera.Handle, "FrameStartTriggerMode", "Freerun");
//
//                // Create infinite loop - break out when condition is met
//                for (;;) {
//                    if (!PvCaptureQueueFrame(myCamera.Handle, &(myCamera.Frame), NULL)) {
//                        double time = (double)getTickCount();
//
//                        while (PvCaptureWaitForFrameDone(myCamera.Handle, &(myCamera.Frame), 100) == ePvErrTimeout) {
//                        }
//
//                        ////////////////////////////////////////////////////////
//                        // Here comes the OpenCV functionality for each frame //
//                        ////////////////////////////////////////////////////////
//
//                        // Create an image header (mono image)
//                        // Push ImageBuffer data into the image matrix
//                        Mat image = Mat(frame_heigth, frame_width, CV_8UC1);
//                        image.data = (uchar *)myCamera.Frame.ImageBuffer;
//
//                        // Show the actual frame
//                        imshow("View window", image);
//
//                        // Now wait for a keystroke and respond to it
//                        int key = waitKey(5);
//                        if (key == 27) {
//                            cout << "Closed down the application by pressing ESC." << endl;
//                            break;
//                        }
//                        if (key == 115) {
//                            cout << "Save key pressed, storing current frame." << endl;
//                            stringstream location;
//                            location << "FILL_IN_A_LOCATION_HERE" << counter << ".png";
//
//                            imwrite(location.str(), image);
//
//                            counter++;
//                        }
//
//                        // Release the image data
//                        image.release();
//                    }
//                }
//
//                // Stop the acquisition & free the camera
//                Errcode = PvCommandRun(myCamera.Handle, "AcquisitionStop");
//                if (Errcode != ePvErrSuccess)
//                    throw Errcode;
//
//                PvCaptureEnd(myCamera.Handle);
//                PvCameraClose(myCamera.Handle);
//
//                cout << endl << "finished" << endl;
//            } else
//                cout << "open camera error" << endl;
//        } else
//            cout << "camera not found" << endl;
//    } else
//        cout << "failed to initialise the API" << endl;
//
//    return 0;
//}
