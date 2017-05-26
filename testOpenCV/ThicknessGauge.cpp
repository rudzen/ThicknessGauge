#include <algorithm>
#include <array>
#include "ThicknessGauge.h"
#include "Calc/MiniCalc.h"
#include "Util/Util.h"
#include "IO/ImageSave.h"
#include "Exceptions/CaptureFailException.h"
#include "Testing/TestConfig.h"
#include "UI/ProgressBar.h"
#include "LineSparse.h"
#include "Histogram/Histogram.h"
#include "IO/GlobGenerator.h"
#include "Histogram/HistoPeak.h"
#include "CV/CannyR.h"
#include "CV/HoughLinesR.h"
#include "CV/Pixel.h"
#include "CV/FilterR.h"
#include "CV/HoughLinesPR.h"
#include "CV/SparseR.h"
#include "Calc/LineCalc.h"
#include <opencv2/core/base.hpp>
#include <opencv2/core/base.hpp>
#include "UI/DrawHelper.h"
#include "CV/GenericCV.h"

/**
 * \brief Initializes the capture device using PV_API constant
 * (requires that OpenCV is compiled with the location of the PvAPI, deprecated version)
 */
void ThicknessGauge::initVideoCapture() {
	cap.open(CV_CAP_PVAPI);
}

/**
 * \brief Initializes the calibration settings
 * \param fileName The filename for the calibration settings
 */
void ThicknessGauge::initCalibrationSettings(string fileName) {
	cs.readSettings(fileName);
}

/**
 * \brief Generates a custom glob
 * \param name The name of the glob, let it be a valid foldername!!!
 */
void ThicknessGauge::generateGlob(std::string& name) {
	if (!cap.isOpened()) // check if we succeeded
		throw CaptureFailException("Error while attempting to open capture device.");

	Util::createDirectory(name);
	auto pbTitle = "Captuing glob " + name;

	ProgressBar pb(frameCount_ * 2, pbTitle.c_str());
	pb.SetFrequencyUpdate(10);
	pb.SetStyle("-", " ");

	cv::Mat t;

	unsigned long progress = 1;
	for (auto i = 0; i < frameCount_; ++i) {
		pb.Progressed(progress++);
		cap >> t;
		pb.Progressed(progress++);
		cv::imwrite(name + "/img" + to_string(i) + ".png", t);
	}
	pb.Progressed(frameCount_ * 2);
}

/**
 * \brief Determins the marking boundries
 * \param globName if "camera", use camera, otherwise load from glob folder
 * \return 2 Float vector with the points marking the boundries as pair, where first = left, second = right
 */
void ThicknessGauge::computeMarkingHeight(std::string& globName) {

	try {

		// determin where to get the frames from.
		if (globName == "camera")
			captureFrames();
		else
			loadGlob(globName);

		// configure frames based on center vertical splitting of the original frames
		//vector<cv::Mat> leftFrames(frameCount_);
		//vector<cv::Mat> rightFrames(frameCount_);

		//splitFrames(leftFrames, rightFrames);

		// common canny with default settings for detecting marking borders
		auto canny = make_shared<CannyR>(200, 250, 3, true, showWindows_, false);

		// the filter used to determin the marking location in the frame
		auto markingFilter = make_shared<FilterR>("Marking Filter");

		// filter to enhance the base line
		auto baselineFilter = make_shared<FilterR>("Baseline Filter");

		// houghlines to determin where the actual marking is in the frame
		auto houghV = make_shared<HoughLinesR>(1, static_cast<const int>(CV_PI / 180), 40, showWindows_);

		// configure the diffrent functionalities
		houghV->setAngleLimit(30);
		markingFilter->setShowWindow(showWindows_);
		baselineFilter->setShowWindow(showWindows_);

		// the marking rect determins where in the frame the marking is located.
		cv::Rect2d markingRect = computerMarkingRectangle(canny, markingFilter, houghV);

		// make sure the minimum is at least 10 pixels.
		auto minLineLen = computeHoughPMinLine<10>(markingRect);

		// horizontal houghline class
		auto houghH = make_shared<HoughLinesPR>(1, cvRound(CV_PI / 180), 40, minLineLen, showWindows_);

		// morph extension class for easy use
		auto morph = make_shared<MorphR>(cv::MORPH_GRADIENT, 1, showWindows_);

		houghH->setMaxLineGab(12);
		houghH->setMarkingRect(markingRect);

		// the baselines, which are located outside the marking
		cv::Vec4f baseLines;

		computeBaseLineAreas(canny, baselineFilter, houghH, morph, baseLines);

		std::cout << cv::format("Base line Y [left] : %f\n", baseLines[1]);
		std::cout << cv::format("Base line Y [right]: %f\n", baseLines[2]);

		// the locations for where the base lines intersect with the marking border
		cv::Vec4f intersections;

		LineCalc lineCalc;

		// compute the intersection points based on the borders of the markings and the baseline for the laser outside the marking
		lineCalc.computeIntersectionPoints(baseLines, houghV->getLeftBorder(), houghV->getRightBorder(), intersections);

		std::cout << "intersection points: " << intersections << std::endl;

		// pixel cut off is based on the border of the marking..
		cv::Vec2f intersectionCutoff = computeIntersectionCut(houghV);

		lineCalc.adjustMarkingRect(markingRect, intersections, intersectionCutoff[0]);

		// adjust the baselines according to the intersection points. (could perhaps be useful in the future)
		lineCalc.adjustBaseLines(baseLines, intersections, intersectionCutoff[0]);

		std::cout << cv::format("Adjusted marking rect: [x: %f | y: %f | w: %f | h: %f]\n", markingRect.x, markingRect.y, markingRect.width, markingRect.height);
		std::cout << cv::format("Adjusted base line Y [left] : %f\n", baseLines[1]);
		std::cout << cv::format("Adjusted base line Y [right]: %f\n", baseLines[3]);

		// work on laser location on marking..
		std::vector<cv::Point2f> laserLine;

		// filter for laser detection
		auto laserFilter = make_shared<FilterR>("Laser Filter");
		laserFilter->setShowWindow(showWindows_);

		// main laser class
		auto laser = make_shared<LaserR>();

		// computes the Y locations of the laserline inside the marking rect
		computeLaserLocations(laser, baseLines, laserFilter, markingRect, laserLine);

		draw->removeAllWindows();

		if (draw->isEscapePressed(30))
			return;

	}
	catch (cv::Exception& e) {
		cerr << cv::format("Exception caught in computeMarkingHeight().\n%s\n", e.msg);
	}

}

/**
 * \brief Computes the base line areas and determine the actual base line.
 * \param canny The canny filter class
 * \param filter The custom filter class
 * \param hough The houghlines class
 * \param morph The morphology class
 * \param output 4-sized float vector descriping the base line locations in x/y with index 0 + 1 = left side and 2 + 3 = right side
 */
void ThicknessGauge::computeBaseLineAreas(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, shared_ptr<MorphR> morph, cv::Vec4f& output) {

	cv::Mat lineHKernel = (cv::Mat_<char>(4, 1) <<
		0 ,
		1 ,
		1 ,
		0
	);

	filter->setKernel(lineHKernel);

	morph->setMethod(cv::MORPH_GRADIENT);
	morph->setIterations(1);

	const std::string leftWindow = "test baseline left";
	const std::string rightWindow = "test baseline right";

	draw->makeWindow(leftWindow);
	draw->makeWindow(rightWindow);

	auto quarter = static_cast<float>(frames[0].rows / 4);
	auto baseLineY = frames[0].rows - quarter;

	auto markingRect = hough->getMarkingRect();

	cv::Rect2f leftBaseLine;
	leftBaseLine.x = 0.0f;
	leftBaseLine.y = baseLineY;
	leftBaseLine.width = markingRect.x;
	leftBaseLine.height = quarter;

	cv::Rect2f rightBaseLine;
	rightBaseLine.x = leftBaseLine.width + markingRect.width;
	rightBaseLine.y = baseLineY;
	rightBaseLine.width = frames[0].cols - rightBaseLine.x;
	rightBaseLine.height = leftBaseLine.height;

	vector<cv::Mat> left;
	vector<cv::Mat> right;

	// generate baseline images..
	for (auto i = frameCount_; i--;) {
		left.push_back(frames[i](leftBaseLine));
		right.push_back(frames[i](rightBaseLine));
	}

	auto imSize = left.front().size();

	auto lineY = 0.0f;

	std::vector<cv::Point2d> tmp;

	std::vector<cv::Point2f> allElements;

	allElements.reserve(imSize.width * imSize.height);

	auto running = true;

	while (running) {

		allElements.clear();

		cv::Mat org;
		// left
		for (auto& l : left) {
			org = l.clone();

			filter->setImage(org);
			filter->doFilter();

			canny->setImage(filter->getResult());
			canny->doCanny();

			morph->setImage(canny->getResult());
			morph->doMorph();

			hough->setImage(morph->getResult());
			hough->setOriginal(org);
			hough->doHorizontalHough();

			const auto& lines = hough->getRightLines();
			for (auto& h : lines)
				Util::copyVector(h.elements, allElements);

			if (draw->isEscapePressed(30)) {
				running = false;
				break;
			}

		}

		// generate real boundry
		auto boundry = cv::minAreaRect(allElements);
		auto boundryRect = boundry.boundingRect2f();

		if (showWindows_) {
			// generate boundry off the elements
			//cv::Point2f vertices[4];
			//boundry.points(vertices);
			//for (auto i = 0; i < 4; ++i)
			//	draw->drawLine(org, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 0, 0));

			draw->drawRectangle(org, boundryRect, cv::Scalar(255, 255, 255));
			draw->showImage(leftWindow, org);
			//draw->showImage(rightWindow, right);
			if (draw->isEscapePressed(30))
				running = false;
		}

		boundryRect.width -= 40;

		auto t = org(boundryRect);
		lineY = frames.front().rows - quarter + boundryRect.y + LineCalc::computeRealIntensityLine(t, tmp, static_cast<float>(t.rows), 0.0f, "_left_baseline", static_cast<float>(frames.front().rows) - quarter + boundryRect.y);

		if (!showWindows_)
			running = false;

		if (running)
			lineY = 0.0f;

	}

	output[0] = 0.0f;
	output[1] = lineY;
	// TODO : Add right side line computation
	// just clone left to right for now
	output[2] = 0.0f;
	output[3] = lineY;

	draw->removeWindow(leftWindow);
	draw->removeWindow(rightWindow);
}

/**
 * \brief Computes the location of the marking rectangle, this rectangle is used to determin the location where the laser is actually on the marking.
 * \param canny The canny filter class
 * \param filter The custom filter class
 * \param hough The houghline class
 * \return The rectangle which was computed
 */
cv::Rect2d ThicknessGauge::computerMarkingRectangle(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesR> hough) {


	const std::string windowName = "test marking out";
	draw->makeWindow(windowName);

	// build cannyfied images
	vector<cv::Mat> candis;
	candis.reserve(frameCount_);

	cv::Mat lineVKernel = (cv::Mat_<char>(4, 4) <<
		0 , 0 , 1 , 1 ,
		0 , 1 , 1 , 1 ,
		1 , 1 , 1 , 0 ,
		1 , 1 , 0 , 0
	);

	filter->setKernel(lineVKernel);

	vector<cv::Mat> cannyImages;

	cv::Mat markingTest;

	vector<cv::Rect2f> markingRects;

	cv::Rect2d output(0.0, 0.0, 0.0, 0.0);

	auto running = true;

	while (running) {

		markingRects.reserve(frameCount_);
		cv::Mat sparse;
		for (auto i = frames.size(); i--;) {
			markingTest = frames.at(i).clone();
			auto org = frames.at(i).clone();
			filter->setImage(frames.at(i).clone());
			filter->doFilter();
			canny->setImage(filter->getResult());
			canny->doCanny();

			auto t = canny->getResult();

			hough->setOriginal(t);
			hough->setImage(t);

			hough->doVerticalHough();
			hough->computeBorders();
			markingRects.push_back(hough->getMarkingRect());
			if (draw->isEscapePressed(30))
				running = false;

		}

		// calculate the avg rect
		output.x = 0.0f;
		output.y = 0.0f;
		output.width = 0.0f;
		output.height = static_cast<float>(frames.front().rows);
		for (auto& r : markingRects) {
			output.x += r.x;
			output.y += r.y;
			output.width += r.width;
		}
		output.x /= markingRects.size();
		output.y /= markingRects.size();
		output.width /= markingRects.size();
		std::cout << "Final marking rect : " << output << endl;

		if (!showWindows_)
			running = false;

		if (!running)
			break;

		if (showWindows_) {
			draw->drawRectangle(markingTest, output, cv::Scalar(128, 128, 128));
			draw->showImage(windowName, markingTest);
			if (draw->isEscapePressed(30))
				running = false;
		}
	}

	if (showWindows_)
		draw->removeWindow(windowName);

	if (output.width > 0.0f && output.height > 0.0f)
		return cv::Rect2d(output);

	CV_Error(cv::Error::StsBadSize, cv::format("Marking rectangle has bad size : [x:%f] [y:%f] [w:%f] [h:%f]\n", output.x, output.y, output.width, output.height));
}

/**
 * \brief Computes the laser line location on the marking in Y
 * \param laser The laser class
 * \param baseLine The baseline vector
 * \param filter The custom filter class
 * \param markingLocation The marking location rectangle
 * \param result The resulting laser line centroid points, with one for each x based on the weigth of their intensity for each X
 */
void ThicknessGauge::computeLaserLocations(shared_ptr<LaserR> laser, cv::Vec4f& baseLine, shared_ptr<FilterR> filter, cv::Rect2d& markingLocation, std::vector<cv::Point2f>& result) {

	// generate frames with marking
	std::vector<cv::Mat> markingFrames;

	for (auto& frame : frames)
		markingFrames.push_back(frame(markingLocation));

	const std::string windowName = "test height";
	draw->makeWindow(windowName);

	// local copy of real baseline
	auto base = frames.front().rows - baseLine[1];

	cv::Mat tmpOut;

	std::vector<cv::Point2d> pixelsOutput;

	auto thresholdLevel = 100.0;

	auto running = true;

	while (running) {

		auto start = cv::getTickCount();

		auto highestPixel = 0.0;

		for (auto i = frameCount_; i--;) {

			cv::Mat baseFrame;

			// TODO : replace with custom filter if needed
			cv::bilateralFilter(markingFrames.at(i), baseFrame, 3, 20, 10);

			//cv::Mat t;
			//GenericCV::adaptiveThreshold(baseFrame, t, &thresholdLevel);

			threshold(baseFrame, baseFrame, thresholdLevel, 255, CV_THRESH_BINARY);

			GaussianBlur(baseFrame, baseFrame, cv::Size(5, 5), 0, 10, cv::BORDER_DEFAULT);

			/* RECT CUT METHOD */
			std::vector<cv::Point> nonZero(baseFrame.rows * baseFrame.cols);
			findNonZero(baseFrame, nonZero);
			auto laserArea = cv::boundingRect(nonZero);
			auto t = baseFrame(laserArea);
			highestPixel += LineCalc::computeRealIntensityLine(t, pixelsOutput, static_cast<float>(t.rows), 0.0f, "_marking", static_cast<float>(laserArea.y));
			highestPixel += (laserArea.y);

			if (draw->isEscapePressed(30))
				running = false;

			if (!i && running)
				cv::cvtColor(markingFrames.at(i), tmpOut, CV_GRAY2BGR);
		}

		auto highestPixelTotal = frames.front().rows - (highestPixel / static_cast<unsigned int>(frameCount_));
		auto end = cv::getTickCount();
		std::cout << cv::format("highestPixelTotal: %f\n", highestPixelTotal);

		auto diff = abs(base - highestPixelTotal);
		std::cout << cv::format("diff from baseline: %f\n", diff);

		auto time = (end - start) / cv::getTickFrequency();
		std::cout << cv::format("time for laser detection (s) : %f\n", time);

		if (!showWindows_)
			running = false;

		if (running && showWindows_) {
			draw->drawHorizontalLine(&tmpOut, cvRound(highestPixelTotal), cv::Scalar(0, 255, 0));
			draw->drawHorizontalLine(&tmpOut, cvRound(base), cv::Scalar(0, 0, 255));
			draw->drawText(&tmpOut, cv::format("%f pixels", diff), TextDrawPosition::UpperLeft);
			draw->drawText(&tmpOut, cv::format("%f s", time), TextDrawPosition::UpperRight);
			draw->showImage(windowName, tmpOut);
			if (draw->isEscapePressed(30))
				running = false;
		}

	}

	draw->removeWindow(windowName);

}

cv::Vec2f ThicknessGauge::computeIntersectionCut(shared_ptr<HoughLinesR> hough) {
	return cv::Vec2f(40.0f, 40.0f);
}

/**
 * \brief Computes the minimum houghline lenght for properlistic houghline
 * \tparam minLen The minimim length of the line
 * \param rect The rectangle of the marking location
 * \return the computed value, but not less than minLen
 */
template <int minLen>
int ThicknessGauge::computeHoughPMinLine(cv::Rect2d& rect) const {
	auto minLineLen = cvRound(rect.width / 32);

	if (minLineLen < minLen)
		minLineLen = minLen;

	return minLineLen;
}

/**
 * \brief Split the original frames into two vectors based on the center of the matrix size in X.
 * Note that the resulting vectors only contains references to the original frames.
 * \param left The output left side of the frames
 * \param right The output right side of the frames
 */
void ThicknessGauge::splitFrames(vector<cv::Mat>& left, vector<cv::Mat>& right) {

	cv::Point topLeft(0, 0);
	cv::Point buttomLeft(frames.front().cols / 2, frames.front().rows);

	cv::Point topRight(frames.front().cols / 2, 0);
	cv::Point buttomRight(frames.front().cols, frames.front().rows);

	cv::Rect leftRect2I(topLeft, buttomLeft);
	cv::Rect rightRect2I(topRight, buttomRight);

	for (auto& f : frames) {
		left.push_back(f(leftRect2I));
		right.push_back(f(rightRect2I));
	}

}

/**
 * \brief Adds the existing null images to the null_ vector for later substraction
 */
void ThicknessGauge::addNulls() {
	std::vector<cv::String> files;
	cv::String folder = "./nulls/";

	cv::glob(folder, files);

	nulls_.clear();
	nulls_.reserve(files.size());

	for (auto& file : files) {
		std::cout << cv::format("loading null file : %s\n", file.c_str());
		nulls_.push_back(cv::imread(file, CV_8UC1));
	}

	for (auto& n : nulls_)
		std::cout << n.size() << endl;

}

/**
 * \brief Loads a glob from disk and stores them in a vector
 * \param globName The name of the glob to load (foldername)
 */
void ThicknessGauge::loadGlob(std::string& globName) {
	globGenerator.setPattern(globName);
	globGenerator.setRecursive(false);
	globGenerator.generateGlob();

	auto files = globGenerator.getFiles();
	auto size = static_cast<int>(files.size());

	if (size != frameCount_)
		setFrameCount(size);

	frames.reserve(size);

	for (auto i = 0; i < size; ++i)
		frames.push_back(cv::imread(files.at(i), CV_8UC1));

	setImageSize(frames.at(0).size());

}

/**
 * \brief Capture frameCount_ amount of frames from the capture device and stores them in a vector
 */
void ThicknessGauge::captureFrames() {
	// check if we succeeded
	if (!cap.isOpened())
	CV_Error(cv::Error::StsError, "Error while attempting to open capture device.");

	cv::Mat t;
	for (auto i = 0; i++ < frameCount_;) {
		cap >> t;
		frames.push_back(t);
	}

	setImageSize(t.size());

}

bool ThicknessGauge::saveData(string filename, std::vector<cv::Point2f>& baseLineLeft, std::vector<cv::Point2f>& baseLineRight, std::vector<cv::Point2f>& mainLine) {
	return true;
}

bool ThicknessGauge::savePlanarImageData(string filename, vector<cv::Point>& pixels, cv::Mat& image, double highestY, string timeString, string extraInfo) const {
	// TODO : needs to be updated + documented

	cv::FileStorage fs(filename + ".json", cv::FileStorage::WRITE);

	if (!fs.isOpened()) {
		cerr << "Error while opening " << filename << " for output." << endl;
		return false;
	}

	ostringstream oss;
	oss << Util::getTime();
	std::cout << cv::format("Saving data [frames: %i] [time: %s] [pixels: %i]\n", frameCount_, oss.str(), pixels.size());

	fs << "Original filename" << filename;
	fs << "Saved time" << Util::getTime();
	fs << "Time to compute" << timeString;
	//fs << "Seconds for computation" << static_cast<long>(frameTime_ / tickFrequency_);
	fs << "Highest Y" << highestY;
	fs << "Extra info" << extraInfo;

	fs << "Eroded element count" << static_cast<int>(pixels.size());
	fs << "Eroded elements" << pixels;
	fs << "Eroded image" << image;

	fs.release();

	//ImageSave is(filename, SaveType::Image_Png);
	//is.SaveImage(image);

	return true;
}

void ThicknessGauge::laplace(cv::Mat& image) const {
	cv::Mat tmp;
	Laplacian(image, tmp, settings.ddepth, settings.kernelSize); // , scale, delta, BORDER_DEFAULT);
	convertScaleAbs(tmp, image);
}

void ThicknessGauge::sobel(cv::Mat& image) const {
	Sobel(image, image, -1, 1, 1, settings.kernelSize, settings.scale, settings.delta, cv::BORDER_DEFAULT);
}

/**
 * \brief Sums the intensity for a specific coloumn in a matrix
 * \param image The image matrix to sum from
 * \param x The X column to sum
 * \return The avg intensity for specified column
 */
double ThicknessGauge::sumColumn(cv::Mat& image, int x) {

	auto sum = 0;
	auto count = 0;

	for (auto col = 0; col < image.cols; ++col) {
		auto uc_pixel = image.data + x * image.step;
		int intensity = uc_pixel[0];
		if (intensity == 0)
			continue;
		sum += intensity;
		count++;
	}

	return sum / static_cast<double>(count);

}

void ThicknessGauge::sumColumns(cv::Mat& image, cv::Mat& target) {

	// note: this function is not the fastest possible,
	// but has security for non-continious image data in matrix

	auto sum = 0;
	for (auto row = 0; row < image.rows; ++row) {
		auto uc_pixel = image.data + row * image.step;
		for (auto col = 0; col < image.cols; ++col) {
			int pixelIntensity = uc_pixel[0];
			sum += pixelIntensity;
			uc_pixel++;
		}
		cout << cv::format("row -> sum: %i -> %i\n", row, sum);
	}
}

void ThicknessGauge::computerGaugeLine(cv::Mat& output) {
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
bool ThicknessGauge::getSparseY(cv::Mat& image, vi& output) const {

	output.reserve(image.cols);

	vi pix;

	pix.reserve(image.cols);

	findNonZero(image, pix);

	// sort the list in X
	sort(pix.begin(), pix.end(), miniCalc.sortX);

	auto x = pix.front().x;
	auto y = 0;
	auto count = 0;
	auto highest = 0;

	for (auto& p : pix) {
		if (p.x != x) {
			if (count > 0) {
				output.push_back(cv::Point(x, y));
				count = 0;
			}
			highest = 0;
		}
		auto intensity = Pixelz::getElementIntensity(image, p);
		if (intensity >= highest) {
			highest = p.y;
			x = p.x;
			y = p.y;
		}
		count++;
	}

	return output.empty() ^ true;

}

int ThicknessGauge::getBinaryThreshold() const {
	return binaryThreshold_;
}

void ThicknessGauge::setBinaryThreshold(int binaryThreshold) {
	binaryThreshold_ = binaryThreshold;
}

int ThicknessGauge::getFrameCount() const {
	return frameCount_;
}

void ThicknessGauge::setFrameCount(int frameCount) {
	if (frameCount < 1 || frameCount > 999)
		frameCount = 25;

	frameCount_ = frameCount;
}

uint64 ThicknessGauge::getFrameTime() const {
	return frameTime_;
}

void ThicknessGauge::setFrameTime(uint64 frameTime) {
	frameTime_ = frameTime;
}

double ThicknessGauge::getTickFrequency() const {
	return tickFrequency_;
}

bool ThicknessGauge::isSaveVideo() const {
	return saveVideo_;
}

void ThicknessGauge::setSaveVideo(bool saveVideo) {
	saveVideo_ = saveVideo;
}

bool ThicknessGauge::isShowWindows() const {
	return showWindows_;
}

void ThicknessGauge::setShowWindows(bool showWindows) {
	showWindows_ = showWindows;
	draw->showWindows(showWindows);
}
