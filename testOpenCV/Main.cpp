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
#include "Vimba/testing/ApiController.h"
#include "Vimba/testing/Bitmap.h"

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

//vimba testing declarations
unsigned char StartsWith(const char* pString, const char* pStart);
int test_vimba(int argc, char* argv[]);

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

    // jump directly into vimba testing for now!
    return test_vimba(argc, argv);

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


unsigned char StartsWith(const char* pString, const char* pStart) {
    if (NULL == pString) {
        return 0;
    }
    if (NULL == pStart) {
        return 0;
    }

    if (std::strlen(pString) < std::strlen(pStart)) {
        return 0;
    }

    if (std::memcmp(pString, pStart, std::strlen(pStart)) != 0) {
        return 0;
    }

    return 1;
}

int test_vimba(int argc, char* argv[]) {
    VmbErrorType err = VmbErrorSuccess;

    char* pCameraID = NULL; // The ID of the camera to use
    const char* pFileName = NULL; // The filename for the bitmap to save
    bool bPrintHelp = false; // Output help?
    int i; // Counter for some iteration
    char* pParameter; // The command line parameter

    std::cout << "//////////////////////////////////////////\n";
    std::cout << "/// Vimba API Synchronous Grab Example ///\n";
    std::cout << "//////////////////////////////////////////\n\n";

    //////////////////////
    //Parse command line//
    //////////////////////

    for (i = 1; i < argc; ++i) {
        pParameter = argv[i];
        if (0 > std::strlen(pParameter)) {
            err = VmbErrorBadParameter;
            break;
        }

        if ('/' == pParameter[0]) {
            if (StartsWith(pParameter, "/f:")) {
                if (NULL != pFileName) {
                    err = VmbErrorBadParameter;
                    break;
                }

                pFileName = pParameter + 3;
                if (0 >= std::strlen(pFileName)) {
                    err = VmbErrorBadParameter;
                    break;
                }
            } else if (0 == std::strcmp(pParameter, "/h")) {
                if ((NULL != pCameraID)
                    || (NULL != pFileName)
                    || (bPrintHelp)) {
                    err = VmbErrorBadParameter;
                    break;
                }

                bPrintHelp = true;
            } else {
                err = VmbErrorBadParameter;
                break;
            }
        } else {
            if (NULL != pCameraID) {
                err = VmbErrorBadParameter;
                break;
            }

            pCameraID = pParameter;
        }
    }

    //Write out an error if we could not parse the command line
    if (VmbErrorBadParameter == err) {
        std::cout << "Invalid parameters!\n\n";
        bPrintHelp = true;
    }

    //Print out help and end program
    if (bPrintHelp) {
        std::cout << "Usage: SynchronousGrab [CameraID] [/h] [/f:FileName]\n";
        std::cout << "Parameters:   CameraID    ID of the camera to use (using first camera if not specified)\n";
        std::cout << "              /h          Print out help\n";
        std::cout << "              /f:FileName File name for operation\n";
        std::cout << "                          (default \"SynchronousGrab.bmp\" if not specified)\n";
    } else {
        if (NULL == pFileName) {
            pFileName = "SynchronousGrab.bmp";
        }

        AVT::VmbAPI::Examples::ApiController apiController;

        std::cout << "Vimba Version V" << apiController.GetVersion() << "\n";

        VmbFrameStatusType status = VmbFrameStatusIncomplete;
        err = apiController.StartUp();
        if (VmbErrorSuccess == err) {
            std::string strCameraID;
            if (NULL == pCameraID) {
                AVT::VmbAPI::CameraPtrVector cameras = apiController.GetCameraList();
                if (cameras.size() <= 0) {
                    err = VmbErrorNotFound;
                } else {
                    err = cameras[0]->GetID(strCameraID);
                }
            } else {
                strCameraID = pCameraID;
            }

            if (VmbErrorSuccess == err) {
                std::cout << "Camera ID:" << strCameraID.c_str() << "\n\n";

                AVT::VmbAPI::FramePtr pFrame;
                err = apiController.AcquireSingleImage(strCameraID, pFrame);
                if (VmbErrorSuccess == err) {
                    err = pFrame->GetReceiveStatus(status);
                    if (VmbErrorSuccess == err
                        && VmbFrameStatusComplete == status) {
                        VmbPixelFormatType ePixelFormat = VmbPixelFormatMono8;
                        err = pFrame->GetPixelFormat(ePixelFormat);
                        if (VmbErrorSuccess == err) {
                            if ((VmbPixelFormatMono8 != ePixelFormat)
                                && (VmbPixelFormatRgb8 != ePixelFormat)) {
                                err = VmbErrorInvalidValue;
                            } else {
                                VmbUint32_t nImageSize = 0;
                                err = pFrame->GetImageSize(nImageSize);
                                if (VmbErrorSuccess == err) {
                                    VmbUint32_t nWidth = 0;
                                    err = pFrame->GetWidth(nWidth);
                                    if (VmbErrorSuccess == err) {
                                        VmbUint32_t nHeight = 0;
                                        err = pFrame->GetHeight(nHeight);
                                        if (VmbErrorSuccess == err) {
                                            VmbUchar_t* pImage = nullptr;
                                            err = pFrame->GetImage(pImage);
                                            if (VmbErrorSuccess == err) {
                                                AVTBitmap bitmap;

                                                if (VmbPixelFormatRgb8 == ePixelFormat) {
                                                    bitmap.colorCode = ColorCodeRGB24;
                                                } else {
                                                    bitmap.colorCode = ColorCodeMono8;
                                                }

                                                bitmap.bufferSize = nImageSize;
                                                bitmap.width = nWidth;
                                                bitmap.height = nHeight;

                                                // test convert to opencv structure


                                                // Create the bitmap
                                                if (0 == AVTCreateBitmap(&bitmap, pImage)) {
                                                    std::cout << "Could not create bitmap.\n";
                                                    err = VmbErrorResources;
                                                } else {
                                                    // Save the bitmap
                                                    if (0 == AVTWriteBitmapToFile(&bitmap, pFileName)) {
                                                        std::cout << "Could not write bitmap to file.\n";
                                                        err = VmbErrorOther;
                                                    } else {
                                                        std::cout << "Bitmap successfully written to file \"" << pFileName << "\"\n";
                                                        // Release the bitmap's buffer
                                                        if (0 == AVTReleaseBitmap(&bitmap)) {
                                                            std::cout << "Could not release the bitmap.\n";
                                                            err = VmbErrorInternalFault;
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            apiController.ShutDown();
        }

        if (VmbErrorSuccess != err) {
            std::string strError = apiController.ErrorCodeToMessage(err);
            std::cout << "\nAn error occurred: " << strError.c_str() << "\n";
        }
        if (VmbFrameStatusIncomplete == status) {
            std::cout << "received frame was not complete\n";
        }
    }

    return err;
}