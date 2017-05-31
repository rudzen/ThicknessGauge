#include <algorithm>
#include <array>
#include "ThicknessGauge.h"
#include "Calc/MiniCalc.h"
#include "Util/Util.h"
#include "IO/ImageSave.h"
#include "Exceptions/CaptureFailException.h"
#include "UI/ProgressBar.h"
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
#include "UI/DrawHelper.h"

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
	auto pbTitle = "Capturing glob " + name;

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

		uint64 startTime = cv::getTickCount();

		// configure frames based on center vertical splitting of the original frames
		//vector<cv::Mat> leftFrames(frameCount_);
		//vector<cv::Mat> rightFrames(frameCount_);
		//splitFrames(leftFrames, rightFrames);

		// common canny with default settings for detecting marking borders
		auto canny = make_shared<CannyR>(200, 250, 3, true, showWindows_, false);

		// the filter used to determin the marking location in the frame
		auto markingFilter = make_shared<FilterR>("Marking Filter", showWindows_);
		markingFilter->setShowWindows(showWindows_);

		// filter to enhance the base line
		auto baselineFilter = make_shared<FilterR>("Baseline Filter", showWindows_);
		baselineFilter->setShowWindows(showWindows_);

		// houghlines to determin where the actual marking is in the frame
		auto houghV = make_shared<HoughLinesR>(1, static_cast<const int>(CV_PI / 180), 40, showWindows_);

		// configure the diffrent functionalities
		houghV->setAngleLimit(30);
		houghV->setShowWindows(showWindows_);

		data->markingRect = computerMarkingRectangle(canny, markingFilter, houghV);
		houghV->setMarkingRect(data->markingRect);

		startTime = cv::getTickCount();

		// check the resulting rectangle for weirdness
		if (data->markingRect.x < 0.0 || data->markingRect.y < 0.0 || data->markingRect.width > imageSize_.width || data->markingRect.height > imageSize_.height || data->markingRect.area() >= imageSize_.area()) {
			CV_Error(cv::Error::StsBadSize, cv::format("Marking rectangle has bad size : [x:%f] [y:%f] [w:%f] [h:%f]\n", data->markingRect.x, data->markingRect.y, data->markingRect.width, data->markingRect.height));
		}

		// make sure the minimum is at least 10 pixels.
		auto minLineLen = computeHoughPMinLine(10.0, data->markingRect);

		// horizontal houghline extension class
		auto houghH = make_shared<HoughLinesPR>(1, cvRound(CV_PI / 180), 40, cvRound(minLineLen), showWindows_);

		houghH->setMaxLineGab(12);
		houghH->setMarkingRect(data->markingRect);
		houghH->setShowWindows(showWindows_);

		// morph extension class
		auto morph = make_shared<MorphR>(cv::MORPH_GRADIENT, 1, showWindows_);

		computeBaseLineAreas(canny, baselineFilter, houghH, morph);
		//std::cout << cv::format("Base line Y [left] : %f\n", data->baseLines[1]);
		//std::cout << cv::format("Base line Y [right]: %f\n", data->baseLines[3]);

		// testing angles
		LineCalc lineCalc;

		// compute the intersection points based on the borders of the markings and the baseline for the laser outside the marking
		lineCalc.computeIntersectionPoints(data->baseLines, houghV->getLeftBorder(), houghV->getRightBorder(), data->intersections);

		std::cout << "intersection points: " << data->intersections << std::endl;

		// pixel cut off is based on the border of the marking..
		cv::Vec2f intersectionCutoff = computeIntersectionCut(houghV);

		lineCalc.adjustMarkingRect(data->markingRect, data->intersections, intersectionCutoff[0]);

		// adjust the baselines according to the intersection points. (could perhaps be useful in the future)
		lineCalc.adjustBaseLines(data->baseLines, data->intersections, intersectionCutoff[0]);

		cv::Point2d leftLine(data->markingRect.x, data->baseLines[1]);
		cv::Point2d rightLine(leftLine.x + data->markingRect.width, data->baseLines[3]);

		cout << "angle between baselines: " << lineCalc.angleBetweenLines(leftLine, rightLine) << endl;

		//std::cout << cv::format("Adjusted marking rect: [x: %f | y: %f | w: %f | h: %f]\n", data->markingRect.x, data->markingRect.y, data->markingRect.width, data->markingRect.height);
		//std::cout << cv::format("Adjusted base line Y [left] : %f\n", data->baseLines[1]);
		//std::cout << cv::format("Adjusted base line Y [right]: %f\n", data->baseLines[3]);

		// filter for laser detection
		auto laserFilter = make_shared<FilterR>("Laser Filter", showWindows_);
		laserFilter->setShowWindows(showWindows_);

		// main laser class
		auto laser = make_shared<LaserR>();

		// computes the Y locations of the laserline inside the marking rect
		computeLaserLocations(laser, laserFilter);

		if (showWindows_)
			draw->removeAllWindows();

		// adjust line points
		for (auto& p : data->leftPoints)
			p.y = imageSize_.height - p.y;
		for (auto& p : data->rightPoints)
			p.y = imageSize_.height - p.y;
		for (auto& p : data->centerPoints)
			p.y = imageSize_.height - p.y;

		uint64 endTime = cv::getTickCount();

		frameTime_ = static_cast<double>((endTime - startTime) / cv::getTickFrequency());

		cout << "Total compute time (seconds) : " << frameTime_ << endl;

		if (draw->isEscapePressed(30))
			return;

	}
	catch (cv::Exception& e) {
		cerr << cv::format("Exception caught in computeMarkingHeight().\n%s\n", e.msg.c_str());
	}

}

/**
 * \brief Computes the base line areas and determine the actual base line.
 * \param canny The canny filter class
 * \param filter The custom filter class
 * \param hough The houghlines class
 * \param morph The morphology class
 */
void ThicknessGauge::computeBaseLineAreas(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, shared_ptr<MorphR> morph) {

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

	if (showWindows_) {
		draw->makeWindow(leftWindow);
		draw->makeWindow(rightWindow);
	}

	auto quarter = static_cast<double>(imageSize_.height) / 4.0;
	auto baseLineY = imageSize_.height - quarter;

	auto markingRect = hough->getMarkingRect();

	cv::Rect2d leftBaseLine;
	leftBaseLine.x = 0.0;
	leftBaseLine.y = baseLineY;
	leftBaseLine.width = markingRect.x;
	leftBaseLine.height = quarter;

	cv::Rect2d rightBaseLine;
	rightBaseLine.x = markingRect.x + markingRect.width;
	rightBaseLine.y = baseLineY;
	rightBaseLine.width = imageSize_.width - rightBaseLine.x;
	rightBaseLine.height = quarter;

	// cannot be resized
	vector<cv::Mat> left;
	vector<cv::Mat> right;

	// generate baseline images..
	for (auto i = frameCount_; i--;) {
		left.emplace_back(frames[i](leftBaseLine));
		right.emplace_back(frames[i](rightBaseLine));
	}

	auto leftSize = left.front().size();
	auto leftCutoff = leftSize.width / 2.0;
	auto rightSize = right.front().size();
	auto rightCutoff = rightSize.width / 2.0;

	auto leftY = 0.0;
	auto rightY = 0.0;

	std::vector<cv::Point2f> leftElements(leftSize.area());
	std::vector<cv::Point2f> rightElements(rightSize.area());

	auto offset = imageSize_.height - quarter;

	auto running = true;

	while (running) {

		leftElements.clear();
		rightElements.clear();

		leftY = 0.0;
		rightY = 0.0;

		cv::Mat org;

		// left

		for (auto& l : left) {
			org = l.clone();
			auto h = l.clone();
			hough->setOriginal(h);

			processMatForLine(org, canny, filter, hough, morph);

			const auto& lines = hough->getRightLines(); // inner most side
			for (auto& h : lines) {
				if (h.entry[0] > leftCutoff)
					Util::copyVector(h.elements, leftElements);
			}

			if (draw->isEscapePressed(30))
				running = false;

		}

		// generate real boundry
		auto leftBoundry = cv::minAreaRect(leftElements);
		auto leftBoundryRect = leftBoundry.boundingRect();

		cout << "rightBoundryRect: " << leftBoundryRect.y << endl;

		leftBoundryRect.width -= 40;

		if (showWindows_) {
			draw->drawRectangle(org, leftBoundryRect, cv::Scalar(255, 255, 255));
			draw->showImage(leftWindow, org);
			if (draw->isEscapePressed(30))
				running = false;
		}

		auto t = org(leftBoundryRect);
		leftY = static_cast<double>(leftBoundryRect.y);
		leftY += offset;
		leftY += LineCalc::computeRealIntensityLine(t, data->leftPoints, t.rows, 0.0);

		cout << "left baseline: " << leftY << endl;

		// right

		for (auto& r : right) {
			org = r.clone();
			auto h1 = r.clone();
			hough->setOriginal(h1);

			processMatForLine(org, canny, filter, hough, morph);

			const auto& lines = hough->getLeftLines(); // inner most side
			for (auto& h : lines) {
				if (h.entry[2] < rightCutoff)
					Util::copyVector(h.elements, rightElements);
			}

			if (draw->isEscapePressed(30))
				running = false;

		}

		// generate real boundry
		auto rightBoundry = cv::minAreaRect(rightElements);
		auto rightBoundryRect = rightBoundry.boundingRect();

		rightBoundryRect.x += 40;

		if (showWindows_) {
			draw->drawRectangle(org, rightBoundryRect, cv::Scalar(255, 255, 255));
			draw->showImage(rightWindow, org);
			if (draw->isEscapePressed(30))
				running = false;
		}

		cout << "rightBoundryRect: " << rightBoundryRect.y << endl;

		t = org(rightBoundryRect);
		rightY = static_cast<double>(rightBoundryRect.y);
		rightY += LineCalc::computeRealIntensityLine(t, data->rightPoints, t.rows, 0);
		rightY += offset;

		cout << "right baseline: " << rightY << endl;

		// forcefully break out of the loop
		if (!showWindows_)
			break;

		running = false;
	}

	data->baseLines[0] = 0.0;
	data->baseLines[1] = leftY;
	data->baseLines[2] = 0.0;
	data->baseLines[3] = rightY;

	if (showWindows_) {
		draw->removeWindow(leftWindow);
		draw->removeWindow(rightWindow);
	}
}

/**
 * \brief Processes the matrix for optimal output and computes the line information based on the results
 * \param org The matrix to perform the process on
 * \param canny The canny extension class used
 * \param filter The filter extension class used
 * \param hough The hough extension class used
 * \param morph The morphology extenstion class used
 */
void ThicknessGauge::processMatForLine(cv::Mat& org, shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, shared_ptr<MorphR> morph) {
	filter->setImage(org);
	filter->doFilter();

	canny->setImage(filter->getResult());
	canny->doCanny();

	morph->setImage(canny->getResult());
	morph->doMorph();

	hough->setImage(morph->getResult());
	hough->doHorizontalHough();
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

	if (showWindows_) {
		draw->makeWindow(windowName);
	}

	cv::Mat lineVKernel = (cv::Mat_<char>(4, 4) <<
		0 , 0 , 1 , 1 ,
		0 , 1 , 1 , 1 ,
		1 , 1 , 1 , 0 ,
		1 , 1 , 0 , 0
	);

	filter->setKernel(lineVKernel);

	cv::Mat markingTest;

	vector<cv::Rect2d> markingRects(frameCount_);
	vector<cv::Vec4d> leftBorders(frameCount_);
	vector<cv::Vec4d> rightBorders(frameCount_);

	cv::Rect2d output(0.0, 0.0, 0.0, 0.0);
	cv::Vec4d leftBorderOut(0.0, 0.0, 0.0, 0.0);
	cv::Vec4d rightBorderOut(0.0, 0.0, 0.0, 0.0);

	auto running = true;

	auto imgHeight = static_cast<double>(imageSize_.height);

	auto accuRects = [imgHeight](vector<cv::Rect2d>& rects, cv::Rect2d& out) {
		out.x = 0.0;
		out.y = 0.0;
		out.width = 0.0;
		out.height = imgHeight;
		for (auto& r : rects) {
			out.x += r.x;
			out.width += r.width;
		}
		out.x /= rects.size();
		out.width /= rects.size();
	};

	auto accuVecs = [imgHeight](vector<cv::Vec4d>& vecs, cv::Vec4d& out) {
		out[0] = 0.0;
		out[1] = imgHeight;
		out[2] = 0.0;
		out[3] = 0.0;
		for (auto& v : vecs) {
			out[0] += v[0];
			out[2] += v[2];
		}
		out[0] /= vecs.size();
		out[2] /= vecs.size();
	};

	while (running) {

		markingRects.clear();
		leftBorders.clear();
		rightBorders.clear();

		cv::Mat sparse;
		for (auto i = frames.size(); i--;) {
			markingTest = frames[i].clone();
			filter->setImage(frames[i].clone());
			filter->doFilter();
			canny->setImage(filter->getResult());
			canny->doCanny();

			auto t = canny->getResult();

			auto tmp = t.clone();
			hough->setOriginal(tmp);
			hough->setImage(t);

			hough->doVerticalHough();
			hough->computeBorders();
			markingRects.emplace_back(hough->getMarkingRect());
			leftBorders.emplace_back(hough->getLeftBorder());
			rightBorders.emplace_back(hough->getRightBorder());
			if (draw->isEscapePressed(30))
				running = false;
		}

		accuRects(markingRects, output);
		accuVecs(leftBorders, leftBorderOut);
		accuVecs(rightBorders, rightBorderOut);

		if (!showWindows_)
			running = false;
		else {
			draw->drawRectangle(markingTest, output, cv::Scalar(128, 128, 128));
			draw->showImage(windowName, markingTest);
			if (draw->isEscapePressed(30))
				running = false;
		}

	}

	if (showWindows_)
		draw->removeWindow(windowName);

	auto validRectangle = [output]()-> bool {
		return output.width > 0.0 && output.height > 0.0;
	};

	bool outputOk = validRectangle();

	if (outputOk) {
		hough->leftBorder(leftBorderOut);
		hough->rightBorder(rightBorderOut);
		return cv::Rect2d(output);
	}

	return cv::Rect2d(0.0, 0.0, 0.0, 0.0);

}

/**
 * \brief Computes the laser line location on the marking in Y
 * \param laser The laser class
 * \param filter The custom filter class
 */
void ThicknessGauge::computeLaserLocations(shared_ptr<LaserR> laser, shared_ptr<FilterR> filter) {

	uint64 startTime = cv::getTickCount();

	// generate frames with marking
	std::vector<cv::Mat> markingFrames;

	for (auto& frame : frames)
		markingFrames.emplace_back(frame(data->markingRect));

	const std::string windowName = "test height";
	if (showWindows_)
		draw->makeWindow(windowName);

	auto imSize = markingFrames.front().size();

	// local copy of real baseline
	auto base = imageSize_.height - data->baseLines[1];

	cv::Mat tmpOut;

	auto running = true;

	std::vector<cv::Point2d> results(imSize.width);

	for (auto i = 0; i < imSize.width; i++)
		results[i].x = i;

	while (running) {

		auto highestPixel = 0.0;

		for (auto i = 0; i < imSize.width; i++)
			results[i].y = 0.0;

		for (auto i = frameCount_; i--;) {

			cv::Mat baseFrame;

			// TODO : replace with custom filter if needed
			cv::bilateralFilter(markingFrames[i], baseFrame, 3, 20, 10);

			//cv::Mat t;
			//GenericCV::adaptiveThreshold(baseFrame, t, &thresholdLevel);

			threshold(baseFrame, baseFrame, binaryThreshold_, 255, CV_THRESH_BINARY);

			GaussianBlur(baseFrame, baseFrame, cv::Size(5, 5), 0, 10, cv::BORDER_DEFAULT);

			/* RECT CUT METHOD */
			std::vector<cv::Point> nonZero(baseFrame.rows * baseFrame.cols);
			findNonZero(baseFrame, nonZero);
			auto laserArea = cv::boundingRect(nonZero);
			auto t = baseFrame(laserArea);
			highestPixel += laserArea.y;
			highestPixel += LineCalc::computeRealIntensityLine(t, data->centerPoints, t.rows, 0);

			for (auto& muffe : data->centerPoints)
				results[static_cast<int>(muffe.x)].y += muffe.y;

			if (draw->isEscapePressed(30))
				running = false;

			if (showWindows_ && !i && running)
				cv::cvtColor(markingFrames[i], tmpOut, CV_GRAY2BGR);
		}

		for (auto& muffe : data->centerPoints)
			results[static_cast<int>(muffe.x)].y /= frameCount_;

		// since theres some issues with using results vector, this works just as fine.
		data->centerPoints.clear();
		Util::copyVector(results, data->centerPoints);

		auto highestPixelTotal = imageSize_.height - highestPixel / static_cast<unsigned int>(frameCount_);

		highestPixel = 0.0;

		std::cout << cv::format("highestPixelTotal: %f\n", highestPixelTotal);

		data->difference = abs(base - highestPixelTotal);
		std::cout << cv::format("diff from baseline: %f\n", data->difference);

		if (!running || !showWindows_)
			break;

		if (showWindows_) {
			draw->drawHorizontalLine(&tmpOut, cvRound(highestPixelTotal), cv::Scalar(0, 255, 0));
			draw->drawHorizontalLine(&tmpOut, cvRound(base), cv::Scalar(0, 0, 255));
			draw->drawText(&tmpOut, cv::format("%f pixels", data->difference), TextDrawPosition::UpperLeft);
			//draw->drawText(&tmpOut, cv::format("%f s", time), TextDrawPosition::UpperRight);
			draw->showImage(windowName, tmpOut);
			if (draw->isEscapePressed(30))
				running = false;
		}
		else {
			break;
		}

	}

	if (showWindows_)
		draw->removeWindow(windowName);

}

cv::Vec2d ThicknessGauge::computeIntersectionCut(shared_ptr<HoughLinesR> hough) {
	auto leftBorder = hough->getLeftBorder();
	auto rightBorder = hough->getRightBorder();


	return cv::Vec2d(40.0, 40.0);
}

/**
 * \brief Computes the minimum houghline lenght for properlistic houghline
 * \tparam minLen The minimim length of the line
 * \param rect The rectangle of the marking location
 * \return the computed value, but not less than minLen
 */
double ThicknessGauge::computeHoughPMinLine(double minLen, cv::Rect2d& rect) {
	auto minLineLen = rect.width / 32.0;

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
		left.emplace_back(f(leftRect2I));
		right.emplace_back(f(rightRect2I));
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

	for (auto& f : files) {
		std::cout << cv::format("loading null file : %s\n", f.c_str());
		nulls_.emplace_back(cv::imread(f, CV_8UC1));
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

	if (files.empty()) {
		CV_Error(cv::Error::StsError, cv::format("No files detected in glob : %s\n", globName));
	}

	auto size = static_cast<int>(files.size());

	if (size != frameCount_)
		setFrameCount(size);

	frames.clear();
	frames.reserve(size);

	for (auto& f : files)
		frames.emplace_back(cv::imread(f, CV_8UC1));

	setImageSize(frames.front().size());

	frames.shrink_to_fit();
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
		frames.emplace_back(t);
	}

	setImageSize(t.size());

}

bool ThicknessGauge::saveData(string filename) {
	auto f(filename + ".json");
	cv::FileStorage fs(f, cv::FileStorage::WRITE);
	if (!fs.isOpened()) {
		cerr << "Error while opening " << f << " for output." << endl;
		return false;
	}

	std::cout << cv::format("Saving data..\n");
	cv::Vec3i sizes(static_cast<int>(data->leftPoints.size()), static_cast<int>(data->centerPoints.size()), static_cast<int>(data->rightPoints.size()));

	fs << "Filename" << filename;
	fs << "TimeSaved" << Util::getTime();
	fs << "ComputeTime" << frameTime_;
	fs << "Difference" << data->difference;
	fs << "MarkingRectangle" << data->markingRect;
	fs << "Intersections" << data->intersections;
	fs << "IntersectionCuts" << data->intersectionCuts;
	fs << "Baselines" << data->baseLines;
	fs << "CenterLine" << data->centerLine;
	fs << "PointSizes" << sizes;
	fs << "ImageSize" << imageSize_;
	fs << "LeftBasePoints" << data->leftPoints;
	fs << "CenterPoints" << data->centerPoints;
	fs << "RightBasePoints" << data->rightPoints;
	fs << "FirstFrame" << frames.front();
	fs.release();

	std::sort(data->leftPoints.begin(), data->leftPoints.end(), miniCalc->sortX);
	std::sort(data->centerPoints.begin(), data->centerPoints.end(), miniCalc->sortX);
	std::sort(data->rightPoints.begin(), data->rightPoints.end(), miniCalc->sortX);

	std::ofstream file(filename + ".1.left.intensitet.txt");

	auto writeY = [&](auto p) { file << p.y << '\n'; };

	// left
	std::for_each(data->leftPoints.begin(), data->leftPoints.end(), writeY);
	file.close();

	// center
	file.open(filename + ".2.center.intensitet.txt");
	std::for_each(data->centerPoints.begin(), data->centerPoints.end(), writeY);
	file.close();

	// right
	file.open(filename + ".2.right.intensitet.txt");
	std::for_each(data->rightPoints.begin(), data->rightPoints.end(), writeY);
	file.close();

	// generate image for output overview and save it.
	cv::Mat overview = cv::Mat::zeros(frames.front().rows, data->leftPoints.size() + data->centerPoints.size() + data->rightPoints.size(), frames.front().type());

	cv::Scalar col(210.0, 210.0, 210.0);

	auto paintY = [](cv::Mat& image, std::vector<cv::Point2d>& points, int offset, cv::Scalar col) {
		for (auto& p : points) {
			image.at<char>(p.y, p.x + offset) = col[0];
		}
	};

	auto offset = 0;
	paintY(overview, data->leftPoints, offset, col);
	offset += static_cast<int>(data->leftPoints.size());
	cv::line(overview, data->leftPoints.back(), cv::Point(data->centerPoints.front().x + offset, data->centerPoints.front().y), col);
	paintY(overview, data->centerPoints, offset, col);
	offset += static_cast<int>(data->centerPoints.size());
	cv::line(overview, cv::Point(data->centerPoints.back().x + data->leftPoints.size(), data->centerPoints.back().y), cv::Point(data->rightPoints.front().x + offset, data->rightPoints.front().y), col);
	paintY(overview, data->rightPoints, offset, col);

	cv::imshow("overview", overview);
	cv::waitKey(0);

	cv::imwrite("_overview.png", overview);

	return true;
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
	sort(pix.begin(), pix.end(), miniCalc->sortX);

	auto x = pix.front().x;
	auto y = 0;
	auto count = 0;
	auto highest = 0;

	for (auto& p : pix) {
		if (p.x != x) {
			if (count > 0) {
				output.emplace_back(cv::Point(x, y));
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

double ThicknessGauge::getFrameTime() const {
	return frameTime_;
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
