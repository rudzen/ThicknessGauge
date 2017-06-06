#define _CRT_SECURE_NO_WARNINGS
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ThicknessGauge.h"
#include "Calibrate/CalibrationTest.cpp"
#include "tclap/CmdLine.h"
#include "Exceptions/CaptureFailException.h"
#include "Exceptions/CalibrationException.h"
#include "Exceptions/TestException.h"
#include "ArgClasses/CommandLineOptions.h"
#include "ArgClasses/IntegerConstraint.h"
#include "ArgClasses/FileConstraint.h"
#include "ArgClasses/DemoModeVisitor.h"
#include "ArgClasses/CalibrationModeVisitor.h"
#include "ArgClasses/TestSuitConstraint.h"
#include "ArgClasses/TestModeVisitor.h"
#include "ArgClasses/BuildInfoVisitor.h"
#include "ArgClasses/GlobModeVisitor.h"
#include <VimbaCPP/Include/VimbaSystem.h>
#include "Vimba/GC2450MCamera.h"
#include "Vimba/CameraData.h"

using namespace std;
using namespace TCLAP;

/*
 * Application return codes :
 * 
 * -1	= Argument parsing error
 * -2	= Capture fail
 * -3	= Calibration error
 * -4	= Test mode exception
 */

#define _USE_MATH_DEFINES


#define RUDZ_DEBUG

const string default_camera_calibration_file = "C2450.json";

bool parseArgs(int argc, char** argv, CommandLineOptions& options);

void saveNull(std::string filename) {

	// quick and dirty hack to save null image quickly
	cout << cv::format("Enter delay in seconds before capture to %s\n>", filename);
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

	std::string name;
	CameraData cameraData;
	AVT::VmbAPI::CameraPtrVector cameras;
	AVT::VmbAPI::VimbaSystem& system = AVT::VmbAPI::VimbaSystem::GetInstance();
	if (VmbErrorSuccess == system.Startup()) {
		if (VmbErrorSuccess == system.GetCameras(cameras)) {
			cameraData.parse(cameras);
			//for (auto& c : cameras) {
			//	c->GetName(name);
			//	std::cout <<  name << std::endl;
			//}
			//for (AVT::VmbAPI::CameraPtrVector::iterator iter = cameras.begin();
			//     cameras.end() != iter;
			//     ++iter) {
			//	if (VmbErrorSuccess == (*iter)->GetName(name)) {
			//		std::cout << name << std::endl;
			//	}
			//}
		}
	}
	system.Shutdown();
	cout << cameraData << endl;
	return 0;

	auto return_value = false;
	CommandLineOptions options;

	try {

		if (parseArgs(argc, argv, options)) {
			// unique case for build information
			if (options.isBuildInfoMode()) {
				Util::log(cv::getBuildInformation());
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

		//cout << options << endl;

		if (options.isGlobMode()) {
			// TODO : use capture for file reading?!
			thicknessGauge->initVideoCapture();
			auto globName = options.getGlobFolder();
			thicknessGauge->generateGlob(globName);
		}
		else if (options.isDemoMode()) {// && !options.TestMode() && !options.CalibrationMode()) {

			auto glob_name = options.getGlobFolder();

			thicknessGauge->initialize(glob_name);

			thicknessGauge->computeMarkingHeight();

			auto data = thicknessGauge->getData(); // virker :-)

			thicknessGauge->saveData("output_mufmuf");

			cout << "difference: " << data->difference << endl;


			//auto result = c.findMarkingLinePairs_(globName);
			//cout << result << endl;
			//returnValue = result.first.x != 0;
			//returnValue = c.generatePlanarImage(globName);
			//if (returnValue) {
			cout << "done..\n";
			//}
		}
		else if (options.isCalibrationMode()) {
			throw CalibrationException("Unable to initiate calibration mode, feature not completed.");
		}
		else if (options.isTestMode()) {
			//c.initVideoCapture();
			//c.testAggressive();
		}
	}
	catch (ArgException& ae) {
		string what = ae.what();
		Util::loge("Exception suddenly happend (but what?)\n" + what);
		return -1;
	}
	catch (CaptureFailException& cfe) {
		string what = cfe.what();
		Util::loge("Exception suddenly happend (but what?)\n" + what);
		return -2;
	}
	catch (CalibrationException& cale) {
		string what = cale.what();
		Util::loge("Exception suddenly happend (but what?)\n" + what);
		return -3;
	}
	catch (TestException& te) {
		string what = te.what();
		Util::loge("Exception suddenly happend (but what?)\n" + what);
		return -4;
	}
	Util::log("\nSmooth operator...");

	cout << return_value << endl;

	return 0;
	//return returnValue ^ true;

	//return testBazier();

	//PrettyMenu menu;
	//menu.show();

	//string calibrate("calibrate");

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
		}
		else if (switch_glob.isSet())
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
	}
	catch (ArgException& e) {
		throw(e);
	}
}

//void test_video(cv::Mat input) {
//
//	// Lambda Matrix
//	Mat lambda(2, 4, CV_32FC1);
//	//Input and Output Image;
//	Mat output;
//
//	// Set the lambda matrix the same type and size as input
//	lambda = Mat::zeros(input.rows, input.cols, input.type());
//
//	// The 4 points that select quadilateral on the input , from top-left in clockwise order
//	// These four pts are the sides of the rect box used as input 
//	// Input Quadilateral or Image plane coordinates
//	Point2f inputQuad[4];
//	CV::GenerateInputQuad(&input, inputQuad);
//
//	// Output Quadilateral or World plane coordinates
//	Point2f outputQuad[4];
//	CV::GenerateOutputQuad(&input, outputQuad);
//
//	CV::drawCenterAxisLines(&input);
//
//	Mat diffImage, doubleDiffImage;
//	Mat outputInverted;
//
//	// configure ImageSave for video output.
//	ImageSave is;
//	is.SetInformation(Information::Full);
//	is.SetSaveType(SaveType::Video);
//	is.SetCodec(VideoCodec::Mjpeg);
//	is.SetFPS(1.0f);
//	is.SetSize(input.cols, input.rows);
//	is.SetColour(VideoColour::Colour);
//	is.SetFileName("Test_numero_uno_pelikan");
//	is.OpenVideo();
//
//	CV::FitQuad(&input, inputQuad, outputQuad);
//
//	float ff;
//
//	const int max = 360;
//
//	int q1 = 0;
//
//	for (int i = 0; i < max; ++i) {
//		ff = static_cast<float>(i);
//		//inputQuad[0] = Point2f(-30.0f, -60.0f);
//		//inputQuad[1] = Point2f(input.cols + 50.0f, -50.0f);
//		//inputQuad[2] = Point2f(input.cols + 100.0f, input.rows + 250.0f);
//		//inputQuad[3] = Point2f(-50.0f, input.rows + 50.0f);
//
//		//// The 4 points where the mapping is to be done , from top-left in clockwise order
//		//outputQuad[0] = Point2f(0, 0);
//		//outputQuad[1] = Point2f(input.cols - 1.0f, 0.0f);
//		//outputQuad[2] = Point2f(input.cols - 1.0f, input.rows - 1.0f);
//		//outputQuad[3] = Point2f(0, input.rows - 1.0f);
//
//		// Get the Perspective Transform Matrix i.e. lambda 
//		lambda = getPerspectiveTransform(outputQuad, inputQuad);
//
//		WRITELOG(logger, framework::Diagnostics::LogLevel::Info, _T("Warping perspective."));
//
//		// Apply the Perspective Transform just found to the src image
//		warpPerspective(input, output, lambda, output.size(), INTER_CUBIC);
//
//		WRITELOG(logger, framework::Diagnostics::LogLevel::Info, _T("Reversing warp."));
//
//		// invert lambda and "recreate" original image
//		warpPerspective(output, outputInverted, lambda.inv(), outputInverted.size(), INTER_LANCZOS4);
//
//		WRITELOG(logger, framework::Diagnostics::LogLevel::Info, _T("Finding leftover artifacts."));
//
//		// get differences between original and restored original image and resize it
//		absdiff(input, outputInverted, diffImage);
//		pyrUp(diffImage, doubleDiffImage);
//		//resize(diffImage, doubleDiffImage, Size(), 2.0f, 2.0f, INTER_CUBIC);
//
//		is.SaveVideoFrame(output);
//
//		//is.UpdateTimeStamp();
//		//is.SaveImage(&output, "OutputTest " + std::to_string(i));
//
//		ff = (static_cast<float>(i) / max) * 100;
//		cout << "                                                            " << '\xd';
//		cout << '\xd' << ff << "%";
//
//	}
//
//	is.CloseVideo();
//
//}
