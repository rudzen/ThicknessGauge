#include <opencv2/opencv.hpp>
#include <iostream>
#include "ThicknessGauge.h"
#include "CalibrationTest.cpp"
#include "args.h"
#include "CaptureFailException.h"

using namespace std;

#define _USE_MATH_DEFINES

RNG rng(12345);

const string default_camera_calibration_file = "C2450.json";

int testBazier() {
	// Create black empty images
	Mat image = Mat::zeros(500, 800, CV_8UC3);

	Point2i Lp(0, 0);

	vector<Point2i> P(20);
	vector<Point2i> Pp(20);
	vector<Point2i> Ppp(20);
	Point2i a(0, 0), b(0, 0), c(0, 0), d(33, 0), e(0, 1), f(0, 0);// general purpose scratch registers.

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

		line(image, c, Lp, Scalar(110, 220, 0), 1, 1);
		Lp = c;
	}

	// P straight line generator.
	for (auto c3ccount = 0; c3ccount < Np; c3ccount = c3ccount + 1) {
		line(image, P[c3ccount], P[c3ccount + 1], Scalar(200, 200, 200), 2, 8);
		a = P[c3ccount] + e;
		line(image, P[c3ccount], a, Scalar(55, 55, 200), 9, 8);
		a = P[c3ccount + 1] + e;
		line(image, P[c3ccount + 1], a, Scalar(55, 55, 200), 9, 8);
	}

	imshow("Bezier_curve", image);
	waitKey(0);
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

int main(int argc, char** argv) {

	//return testBazier();

	//PrettyMenu menu;
	//menu.show();

	string calibrate("calibrate");

	Args args;
	args.addOption("demo", 'd', "start basic demo", Args::Optional, false);
	args.addOption("calibrate", 'c', "Calibrates camera a single frame at a time", Args::Optional, false);
	args.addOption("frames", 'f', "Frame interval between computations", Args::Optional, false);
	args.addOption("show", 's', "Show OpenCV output windows", Args::Optional, false);
	args.addOption("buildinfo", 'b', "Show OpenCV build information", Args::Optional, false);
	args.addOption("settings", 'l', "Loads camera calibration settings from file", Args::Optional, false);
	args.addOption("video", 'v', "Enable video recording of resulting images.", Args::Optional, false);
	//	args.addAdditionalOption(calibrate, "Calibration method");
	args.parse(argc, argv);

	auto showBuildInfo = args.getArgument("buildinfo", false);

	if (showBuildInfo) {
		cout << getBuildInformation();
		return 0;
	}

	auto initiateCalibration = args.getArgument("calibrate", false);

	if (initiateCalibration) {
		// TODO : Construct modular calibration test and start it here!
		cout << "Calibration is not completed yet, please do so manually." << endl;
		return 0;
	}

	ThicknessGauge c;

	// settings from arguments.
	c.setFrameCount(args.getArgument("frames", 25));
	c.setShowWindows(args.getArgument("show", true));
	c.setSaveVideo(args.getArgument("video", false));

	auto camera_calibration_file = args.getArgument("settings", default_camera_calibration_file);

	auto calibration_file_exists = Util::isFile(camera_calibration_file);

	if (calibration_file_exists) {
		c.initCalibrationSettings(camera_calibration_file);
	}
	else {
		cerr << "Error. Calibration file does not exist, no settings will be applied." << endl;
	}

	c.initVideoCapture();

	auto returnValue = false;
	auto bThreshold = 0;
	try {
		//bThreshold = c.autoBinaryThreshold(25000);
		//cout << "generating threshold : " << bThreshold << endl;
		//returnValue = bThreshold != 0;

		returnValue = c.generatePlanarImage();
		if (returnValue) {
			cout << "Planar image generated in " << c.getFrameTime() / c.getTickFrequency() << " seconds, processing..\n";
		}
	}
	catch (CaptureFailException& e) {
		cout << "Something happend.. but what?\n" << e.what() << endl;
	}

	return returnValue ^ true;

}
