#pragma once

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Util/Util.h"
#include "Util/Stringtools.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace std;
using namespace utils;

class Settings {

public:

	Settings() : calibrationPattern(), squareSize(0), nrFrames(0), aspectRatio(0), delay(0), bwritePoints(false), bwriteExtrinsics(false), calibZeroTangentDist(false), calibFixPrincipalPoint(false), flipVertical(false), showUndistorsed(false), cameraID(0), atImageList(0), inputType(), goodInput(false), flag(0) {
	}

	enum class Mode { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

	enum class Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

	enum class InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

	//Write serialization for this class
	void write(cv::FileStorage& fs) const {
		fs << "{" << "BoardSize_Width" << boardSize.width
			<< "BoardSize_Height" << boardSize.height
			<< "Square_Size" << squareSize
			<< "Calibrate_Pattern" << patternToUse
			<< "Calibrate_NrOfFrameToUse" << nrFrames
			<< "Calibrate_FixAspectRatio" << aspectRatio
			<< "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
			<< "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

			<< "Write_DetectedFeaturePoints" << bwritePoints
			<< "Write_extrinsicParameters" << bwriteExtrinsics
			<< "Write_outputFileName" << outputFileName

			<< "Show_UndistortedImage" << showUndistorsed

			<< "Input_FlipAroundHorizontalAxis" << flipVertical
			<< "Input_Delay" << delay
			<< "Input" << input
			<< "}";
	}

	//Read serialization for this class
	void read(const cv::FileNode& node) {
		node["BoardSize_Width"] >> boardSize.width;
		node["BoardSize_Height"] >> boardSize.height;
		node["Calibrate_Pattern"] >> patternToUse;
		node["Square_Size"] >> squareSize;
		node["Calibrate_NrOfFrameToUse"] >> nrFrames;
		node["Calibrate_FixAspectRatio"] >> aspectRatio;
		node["Write_DetectedFeaturePoints"] >> bwritePoints;
		node["Write_extrinsicParameters"] >> bwriteExtrinsics;
		node["Write_outputFileName"] >> outputFileName;
		node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
		node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
		node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
		node["Show_UndistortedImage"] >> showUndistorsed;
		node["Input"] >> input;
		node["Input_Delay"] >> delay;
		interprate();
	}

	void interprate() {
		goodInput = true;
		if (boardSize.width <= 0 || boardSize.height <= 0) {
			cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
			goodInput = false;
		}
		if (squareSize <= 10e-6) {
			cerr << "Invalid square size " << squareSize << endl;
			goodInput = false;
		}
		if (nrFrames <= 0) {
			cerr << "Invalid number of frames " << nrFrames << endl;
			goodInput = false;
		}

		if (input.empty()) // Check for valid input
			inputType = InputType::INVALID;
		else {
			if (input[0] >= '0' && input[0] <= '9') {
				stringstream ss(input);
				ss >> cameraID;
				inputType = InputType::CAMERA;
			}
			else {
				if (isListOfImages(input) && readStringList(input, imageList)) {
					inputType = InputType::IMAGE_LIST;
					nrFrames = (nrFrames < static_cast<int>(imageList.size())) ? nrFrames : static_cast<int>(imageList.size());
				}
				else
					inputType = InputType::VIDEO_FILE;
			}
			if (inputType == InputType::CAMERA)
				inputCapture.open(cameraID);
			if (inputType == InputType::VIDEO_FILE)
				inputCapture.open(input);
			if (inputType != InputType::IMAGE_LIST && !inputCapture.isOpened())
				inputType = InputType::INVALID;
		}

		if (inputType == InputType::INVALID) {
			cerr << " Inexistent input: " << input;
			goodInput = false;
		}

		flag = 0;
		if (calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
		if (calibZeroTangentDist) flag |= CV_CALIB_ZERO_TANGENT_DIST;
		if (aspectRatio) flag |= CV_CALIB_FIX_ASPECT_RATIO;

		calibrationPattern = Pattern::NOT_EXISTING;
		if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = Pattern::CHESSBOARD;
		if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = Pattern::CIRCLES_GRID;
		if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = Pattern::ASYMMETRIC_CIRCLES_GRID;

		if (calibrationPattern == Pattern::NOT_EXISTING) {
			cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
			goodInput = false;
		}

		atImageList = 0;
	}

	cv::Mat nextImage() {
		cv::Mat result;
		if (inputCapture.isOpened()) {
			cv::Mat view0;
			inputCapture >> view0;
			return view0;
			//view0.copyTo(result);
		}
		if (atImageList < static_cast<int>(imageList.size()))
			result = cv::imread(imageList[atImageList++], CV_LOAD_IMAGE_COLOR);

		return result;
	}

	static bool readStringList(const string& filename, vector<string>& l) {
		l.clear();
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		if (!fs.isOpened())
			return false;
		auto node = fs.getFirstTopLevelNode();
		if (node.type() != cv::FileNode::SEQ)
			return false;
		auto file_node_iterator = node.begin(), it_end = node.end();
		for (; file_node_iterator != it_end; ++file_node_iterator)
			l.push_back(static_cast<string>(*file_node_iterator));
		return true;
	}

	static bool isListOfImages(const string& filename) {

		return StringTools::endsWith(filename, ".xml") ||
			StringTools::endsWith(filename, ".yaml") ||
			StringTools::endsWith(filename, ".yml") ||
			StringTools::endsWith(filename, ".json");
		//auto s(filename);
		//// Look for file extension
		//if (s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos && string::npos && s.find(".json") == string::npos)
		//	return false;
		//else
		//	return true;
	}

public:
	cv::Size boardSize; // The size of the board -> Number of items by width and height
	Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
	float squareSize; // The size of a square in your defined unit (point, millimeter,etc).
	int nrFrames; // The number of frames to use from the input for calibration
	float aspectRatio; // The aspect ratio
	int delay; // In case of a video input
	bool bwritePoints; //  Write detected feature points
	bool bwriteExtrinsics; // Write extrinsic parameters
	bool calibZeroTangentDist; // Assume zero tangential distortion
	bool calibFixPrincipalPoint;// Fix the principal point at the center
	bool flipVertical; // Flip the captured images around the horizontal axis
	string outputFileName; // The name of the file where to write
	bool showUndistorsed; // Show undistorted images after calibration
	string input; // The input ->

	int cameraID;
	vector<string> imageList;
	int atImageList;
	cv::VideoCapture inputCapture;
	InputType inputType;
	bool goodInput;
	int flag;

	// functions
	static void help() {
		cout << "Camera calibration test tool v0.1.\n"
			<< "Usage: calibration configurationFile\n"
			<< "Near the sample file you'll find the configuration file, which has detailed help of "
			"how to edit it.  It may be any OpenCV supported file format XML/YAML/JSON." << endl;
	}

	static double computeReprojectionErrors(const vector<vector<cv::Point3f>>& objectPoints,
	                                        const vector<vector<cv::Point2f>>& imagePoints,
	                                        const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
	                                        const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	                                        vector<float>& perViewErrors) {
		vector<cv::Point2f> imagePoints2;
		auto totalPoints = 0;
		double totalErr = 0;
		perViewErrors.resize(objectPoints.size());

		for (auto i = 0; i < static_cast<int>(objectPoints.size()); ++i) {
			projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
			auto err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);

			auto n = static_cast<int>(objectPoints[i].size());
			perViewErrors[i] = static_cast<float>(std::sqrt(err * err / n));
			totalErr += err * err;
			totalPoints += n;
		}

		return std::sqrt(totalErr / totalPoints);
	}

	// Print camera parameters to the output file
	void saveCameraParams(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	                      const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
	                      const vector<float>& reprojErrs, const vector<vector<cv::Point2f>>& imagePoints,
	                      double totalAvgErr) const {
		cv::FileStorage fs(s.outputFileName, cv::FileStorage::WRITE_BASE64);

		fs << "calibration_Time" << Util::getTime();

		if (!rvecs.empty() || !reprojErrs.empty())
			fs << "nrOfFrames" << static_cast<int>(max(rvecs.size(), reprojErrs.size()));
		fs << "image_Width" << imageSize.width;
		fs << "image_Height" << imageSize.height;
		fs << "board_Width" << s.boardSize.width;
		fs << "board_Height" << s.boardSize.height;
		fs << "square_Size" << s.squareSize;

		if (s.flag & CV_CALIB_FIX_ASPECT_RATIO)
			fs << "FixAspectRatio" << s.aspectRatio;


		if (s.flag) {
			string out("flags: ");
			if (s.flag & CV_CALIB_USE_INTRINSIC_GUESS)
				out.append(" +use_intrinsic_guess");
			if (s.flag & CV_CALIB_FIX_ASPECT_RATIO)
				out.append(" +fix_aspectRatio");
			if (s.flag & CV_CALIB_FIX_PRINCIPAL_POINT)
				out.append(" +fix_principal_point");
			if (s.flag & CV_CALIB_ZERO_TANGENT_DIST)
				out.append(" +zero_tangent_dist");
			cvWriteComment(*fs, out.c_str(), 0);

			//char buf[1024];
			//sprintf(buf, "flags: %s%s%s%s",
			//        s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
			//        s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
			//        s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
			//        s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
			//cvWriteComment(*fs, buf, 0);

		}

		fs << "flagValue" << s.flag;

		fs << "Camera_Matrix" << cameraMatrix;
		fs << "Distortion_Coefficients" << distCoeffs;

		fs << "Avg_Reprojection_Error" << totalAvgErr;
		if (!reprojErrs.empty())
			fs << "Per_View_Reprojection_Errors" << cv::Mat(reprojErrs);

		if (!rvecs.empty() && !tvecs.empty()) {
			CV_Assert(rvecs[0].type() == tvecs[0].type());
			cv::Mat bigmat(static_cast<int>(rvecs.size()), 6, rvecs[0].type());
			for (auto i = 0; i < static_cast<int>(rvecs.size()); i++) {
				// ReSharper disable CppInitializedValueIsAlwaysRewritten
				auto rotation = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
				auto translation = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));
				// ReSharper restore CppInitializedValueIsAlwaysRewritten

				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				//*.t() is MatExpr (not Mat) so we can use assignment operator
				rotation = rvecs[i].t();
				translation = tvecs[i].t();
			}
			cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
			fs << "Extrinsic_Parameters" << bigmat;
		}

		if (imagePoints.empty())
			return;

		cv::Mat imagePtMat(static_cast<int>(imagePoints.size()), static_cast<int>(imagePoints[0].size()), CV_32FC2);
		for (auto i = 0; i < static_cast<int>(imagePoints.size()); i++) {
			auto r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			cv::Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "Image_points" << imagePtMat;
	}

	bool runCalibration(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	                    vector<vector<cv::Point2f>> imagePoints, vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs,
	                    vector<float>& reprojErrs, double& totalAvgErr) const {

		cout << "Configuring matricies.." << endl;

		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		if (s.flag & CV_CALIB_FIX_ASPECT_RATIO)
			cameraMatrix.at<double>(0, 0) = 1.0;

		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

		cout << "Calculating board corner positions and resizing object points.." << endl;

		vector<vector<cv::Point3f>> objectPoints(1);
		calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

		objectPoints.resize(imagePoints.size(), objectPoints[0]);

		cout << "Configuring camera intrinsic & extrinsic parameters." << endl;

		//Find intrinsic and extrinsic camera parameters
		auto rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		                           distCoeffs, rvecs, tvecs, s.flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

		cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

		auto ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

		return ok;
	}

	static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, vector<cv::Point3f>& corners, Pattern patternType /*= Settings::CHESSBOARD*/) {
		corners.clear();

		switch (patternType) {
		case Pattern::CHESSBOARD:
		case Pattern::CIRCLES_GRID:
			for (auto i = 0; i < boardSize.height; ++i)
				for (auto j = 0; j < boardSize.width; ++j)
					corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
			break;

		case Pattern::ASYMMETRIC_CIRCLES_GRID:
			for (auto i = 0; i < boardSize.height; i++)
				for (auto j = 0; j < boardSize.width; j++)
					corners.push_back(cv::Point3f(float((2 * j + i % 2) * squareSize), float(i * squareSize), 0));
			break;
		default:
			break;
		}
	}

	bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, vector<vector<cv::Point2f>> imagePoints) const {
		vector<cv::Mat> rvecs;
		vector<cv::Mat> tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0;

		auto ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);
		cout << (ok ? "Calibration succeeded" : "Calibration failed")
			<< ". avg re projection error = " << totalAvgErr;

		if (ok)
			saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints, totalAvgErr);

		return ok;
	}

	int init(Settings& s, const string inputSettingsFile = "default.xml") const {
		help();

		// Read the settings
		cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ);

		if (!fs.isOpened()) {
			cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
			return -1;
		}

		fs["Settings"] >> s;
		// close Settings file
		fs.release();

		if (!s.goodInput) {
			cout << "Invalid input detected. Stopping calibration. " << endl;
			return -1;
		}

		vector<vector<cv::Point2f>> imagePoints;
		cv::Mat cameraMatrix, distCoeffs;
		cv::Size imageSize;
		auto mode = s.inputType == InputType::IMAGE_LIST ? Mode::CAPTURING : Mode::DETECTION;
		clock_t prevTimestamp = 0;
		const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);
		const char ESC_KEY = 27;

		cv::namedWindow("Image View", cv::WINDOW_FREERATIO);

		for (auto i = 0;; ++i) {
			auto view = s.nextImage();
			auto blinkOutput = false;

			//-----  If no more image, or got enough, then stop calibration and show result -------------
			if (mode == Mode::CAPTURING && imagePoints.size() >= static_cast<unsigned>(s.nrFrames)) {
				if (s.runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints))
					mode = Mode::CALIBRATED;
				else
					mode = Mode::DETECTION;
			}

			// If no more images then run calibration, save and stop loop.
			if (view.empty()) {
				if (imagePoints.size() > 0)
					s.runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints);
				break;
			}

			// Format input image.
			imageSize = view.size();

			if (s.flipVertical)
				flip(view, view, 0);

			vector<cv::Point2f> pointBuf;

			// Find feature points on the input format
			bool found;
			switch (s.calibrationPattern) {
			case Pattern::CHESSBOARD:
				found = findChessboardCorners(view, s.boardSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
				break;
			case Pattern::CIRCLES_GRID:
				found = findCirclesGrid(view, s.boardSize, pointBuf);
				break;
			case Pattern::ASYMMETRIC_CIRCLES_GRID:
				found = findCirclesGrid(view, s.boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
				break;
			default:
				found = false;
				break;
			}

			// If done with success,
			if (found) {
				// improve the found corners' coordinate accuracy for chessboard
				if (s.calibrationPattern == Pattern::CHESSBOARD) {
					// from colour to gray
					cv::Mat viewGray;
					cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
					cornerSubPix(viewGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				}

				// For camera only take new samples after delay time
				if (mode == Mode::CAPTURING && (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay * 1e-3 * CLOCKS_PER_SEC)) {
					imagePoints.push_back(pointBuf);
					prevTimestamp = clock();
					blinkOutput = s.inputCapture.isOpened();
				}

				// Draw the corners.
				drawChessboardCorners(view, s.boardSize, cv::Mat(pointBuf), found);
			}

			//----------------------------- Output Text ------------------------------------------------
			string msg = (mode == Mode::CAPTURING) ? "100/100" : mode == Mode::CALIBRATED ? "Calibrated" : "Press 'g' to start";
			auto baseLine = 0;
			auto textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
			cv::Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

			if (mode == Mode::CAPTURING) {
				if (s.showUndistorsed)
					msg = cv::format("%d/%d Undist", static_cast<int>(imagePoints.size()), s.nrFrames);
				else
					msg = cv::format("%d/%d", static_cast<int>(imagePoints.size()), s.nrFrames);
			}

			putText(view, msg, textOrigin, 1, 1, mode == Mode::CALIBRATED ? GREEN : RED);

			if (blinkOutput)
				bitwise_not(view, view);

			//------------------------- Video capture  output  undistorted ------------------------------
			if (mode == Mode::CALIBRATED && s.showUndistorsed) {
				auto temp = view.clone();
				undistort(temp, view, cameraMatrix, distCoeffs);
			}

			//------------------------------ Show image and check for input commands -------------------
			cout << s.imageList[s.atImageList] << endl;
			imshow("Image View", view);
			auto key = static_cast<char>(cv::waitKey(s.inputCapture.isOpened() ? 50 : s.delay));

			if (key == ESC_KEY)
				break;

			if (key == 'u' && mode == Mode::CALIBRATED)
				s.showUndistorsed ^= true;

			if (s.inputCapture.isOpened() && key == 'g') {
				mode = Mode::CAPTURING;
				imagePoints.clear();
			}
		}

		// -----------------------Show the undistorted image for the image list ------------------------
		if (s.inputType == InputType::IMAGE_LIST && s.showUndistorsed) {
			cv::Mat rview, map1, map2;
			initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, nullptr), imageSize, CV_16SC2, map1, map2);

			for (auto i = 0; i < static_cast<int>(s.imageList.size()); i++) {
				auto view = cv::imread(s.imageList[i], 1);
				if (view.empty())
					continue;

				remap(view, rview, map1, map2, cv::INTER_LINEAR);
				imshow("Image View", rview);
				auto c = static_cast<char>(cv::waitKey());
				if (c == ESC_KEY || c == 'q' || c == 'Q') {
					break;
				}
			}
		}

		return 0;
	}


private:
	string patternToUse;
};

static void read(const cv::FileNode& node, Settings& x, const Settings& default_value = Settings()) {
	if (node.empty())
		x = default_value;
	else
		x.read(node);
}

//bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
//	vector<vector<Point2f> > imagePoints);


//int Settings::readSettings(const string calibrationFile, Mat *cameraMatrix, Mat *distortion, double *avg_error, Mat *perViewReprojectionErrors, Mat *extrinsicParameters, Mat *imagePoints)
//{
//	FileStorage fs;
//	fs.open(calibrationFile, FileStorage::READ);
//
//	if (!fs.isOpened())
//	{
//		cerr << "Failed to open " << calibrationFile << endl;
//		return -1;
//	}
//
//	//cv::FileNode d = fs["data"];
//	fs["Camera_Matrix"] >> *cameraMatrix;
//
//	fs["Distortion_Coefficients"] >> *distortion;
//
//	fs["Avg_Reprojection_Error"] >> *avg_error;
//
//	fs["Per_View_Reprojection_Errors"] >> *perViewReprojectionErrors;
//	fs["Extrinsic_Parameters"] >> *extrinsicParameters;
//
//	fs["Image_points"] >> *imagePoints;
//
//	fs.release();
//
//	return 0;
//}
