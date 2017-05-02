#include <opencv2/opencv.hpp>
#include <iostream>
#include "ThicknessGauge.h"
#include "CalibrationTest.cpp"
#include "CaptureFailException.h"
#include "tclap/CmdLine.h"
#include "CommandLineOptions.h"
#include "IntegerConstraint.h"
#include "FileConstraint.h"
#include "CalibrationException.h"
#include "TestException.h"

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
cv::RNG rng(12345);
const string default_camera_calibration_file = "C2450.json";

bool parseArgs(int argc, char** argv, CommandLineOptions& options);

int main(int argc, char** argv) {

	auto returnValue = false;
	CommandLineOptions options;

	try {

		if (parseArgs(argc, argv, options)) {
			// unique case for build information
			if (options.BuildInfoMode()) {
				Util::log(cv::getBuildInformation());
				return 0;
			}
		}

		ThicknessGauge c;

		c.setFrameCount(options.Frames());
		c.setShowWindows(options.ShowWindows());
		c.setSaveVideo(options.RecordVideo());

		auto calibration_file_exists = Util::isFile(options.CameraFile());
		if (calibration_file_exists) {
			c.initCalibrationSettings(options.CameraFile());
		}

		if (options.DemoMode() && !options.TestMode() && !options.CalibrationMode()) {

			c.initVideoCapture();

			auto bThreshold = 0;
			//bThreshold = c.autoBinaryThreshold(25000);
			//cout << "generating threshold : " << bThreshold << endl;
			//returnValue = bThreshold != 0;

			returnValue = c.generatePlanarImage();
			if (returnValue) {
				cout << "Planar image generated in " << c.getFrameTime() / c.getTickFrequency() << " seconds, processing..\n";
			}
		} else if (options.CalibrationMode()) {
			throw CalibrationException("Unable to initiate calibration mode, feature not completed.");
		} else if (options.TestMode()) {
			c.initVideoCapture();
			c.testAggressive();
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

	Util::log("Smooth operator...");

	return returnValue ^ true;

	//return testBazier();

	//PrettyMenu menu;
	//menu.show();

	//string calibrate("calibrate");

}

bool parseArgs(int argc, char** argv, CommandLineOptions& options) {
	try {
		CmdLine cmd("ThicknessGauge [OpenCV]", '=', "0.1", true);

		// add basic switches
		SwitchArg demoSwitch("d", "demo", "runs regular demo", true);
		cmd.add(demoSwitch);

		SwitchArg calibrationSwitch("c", "calibrate", "perform camera calibration", false);
		cmd.add(calibrationSwitch);

		SwitchArg buildInfoSwitch("i", "info", "show software information", false);
		cmd.add(buildInfoSwitch);

		SwitchArg testSwitch("t", "test_frames", "Performs aggresive testing from 5 to -m=<frames>", false);
		cmd.add(testSwitch);

		ValueArg<int> frameArg("f", "frames", "amount of frames each calculation", false, 25, new IntegerConstraint(5, 200));
		cmd.add(frameArg);

		ValueArg<bool> showArg("s", "show_windows", "displays windows in demo mode", false, true, "true/false");
		cmd.add(showArg);

		ValueArg<bool> videoArg("v", "record_video", "Records demo mode to video", false, false, "true/false");
		cmd.add(videoArg);

		ValueArg<int> testArg("m", "max_frames", "Aggresive testing maximum frames", false, 50, new IntegerConstraint(10, 1000));
		cmd.add(testArg);

		ValueArg<string> cameraFileArg("", "camera_settings", "OpenCV camera calibration file", false, default_camera_calibration_file, new FileConstraint());
		cmd.add(cameraFileArg);

		ValueArg<string> calibrationOutput("", "calibrate_output", "output file for camera matricies", false, "output.json", "filename");
		cmd.add(calibrationOutput);

		cmd.parse(argc, argv);

		// read all parsed command line arguments
		auto buildinfo = buildInfoSwitch.getValue();

		options.setBuildInfoMode(buildinfo);

		// check, its a instant abort if build info is found
		if (buildinfo)
			return true;

		auto cameraFile = cameraFileArg.getValue();
		options.setCameraFile(cameraFile);

		cameraFile = calibrationOutput.getValue();
		options.setCalibrationOutput(cameraFile);

		auto frames = frameArg.getValue();
		options.set_frames(frames);

		auto showwindows = showArg.getValue();
		auto recordvideo = videoArg.getValue();
		auto testmaxframes = testArg.getValue();

		options.setShowWindows(showwindows);
		options.setRecordVideo(recordvideo);
		options.setTestMax(testmaxframes);

		auto demo = demoSwitch.getValue();
		auto calib = calibrationSwitch.getValue();
		auto test = testSwitch.getValue();

		options.setDemoMode(demo);
		options.setCalibrationMode(calib);
		options.setTestMode(test);

		return false;
	}
	catch (ArgException& e) {
		throw(e);
	}
}


int testBazier() {
	// Create black empty images
	cv::Mat image = cv::Mat::zeros(500, 800, CV_8UC3);

	cv::Point2i Lp(0, 0);

	vector<cv::Point2i> P(20);
	vector<cv::Point2i> Pp(20);
	vector<cv::Point2i> Ppp(20);
	cv::Point2i a(0, 0), b(0, 0), c(0, 0), d(33, 0), e(0, 1), f(0, 0);// general purpose scratch registers.

	unsigned int Np = 4;
	P[0] = {133, 300};
	P[1] = {260, 400};
	P[2] = {555, 111};
	P[3] = {700, 111};

	//  P(5) = { , };
	//  P(6) = { , };
	//  P(55) = { , };
	Np--;

	Lp = P[0];
	for (float ppercent = 0.00; ppercent < 1.0; ppercent = ppercent + 0.01) {
		Pp = P;
		int c2ccount = Np;
		for (auto c1ccount = 0; c1ccount < Np; c1ccount = c1ccount + 1) {
			for (auto cccount = 0; cccount < c2ccount; cccount = cccount + 1) {
				a = Pp[cccount] - Pp[cccount + 1];
				a = Pp[cccount + 1] - Pp[cccount];
				b = ppercent * a;
				c = Pp[cccount] + b;
				Ppp[cccount] = c;
			}
			c2ccount = c2ccount - 1;
			Pp = Ppp;
		}

		line(image, c, Lp, cv::Scalar(110, 220, 0), 1, 1);
		Lp = c;
	}

	// P straight line generator.
	for (auto c3ccount = 0; c3ccount < Np; c3ccount = c3ccount + 1) {
		line(image, P[c3ccount], P[c3ccount + 1], cv::Scalar(200, 200, 200), 2, 8);
		a = P[c3ccount] + e;
		line(image, P[c3ccount], a, cv::Scalar(55, 55, 200), 9, 8);
		a = P[c3ccount + 1] + e;
		line(image, P[c3ccount + 1], a, cv::Scalar(55, 55, 200), 9, 8);
	}

	imshow("Bezier_curve", image);
	cv::waitKey(0);
	cout << "done\n";
	return (0);
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
