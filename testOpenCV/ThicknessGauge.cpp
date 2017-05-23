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
#include "CV/LineData/LineBaseData.h"
#include "CV/LineData/LineLaserData.h"
#include "CV/SparseR.h"
#include "Calc/LineCalc.h"
#include <opencv2/core/base.hpp>
#include <opencv2/core/base.hpp>
#include "UI/DrawHelper.h"

void ThicknessGauge::initVideoCapture() {
	cap.open(CV_CAP_PVAPI);
}

void ThicknessGauge::initCalibrationSettings(string fileName) {
	cs.readSettings(fileName);
}

void ThicknessGauge::addNulls() {
	std::vector<cv::String> files;
	cv::String folder = "./nulls/";

	cv::glob(folder, files);

	nulls_.clear();
	nulls_.reserve(files.size());

	for (auto& file : files) {
		std::cout << "loading null file : " << file << endl;
		//cv::Mat tmp = cv::imread(file, CV_8UC1);

		nulls_.push_back(cv::imread(file, CV_8UC1));
	}

	for (auto& n : nulls_)
		std::cout << n.size() << endl;

}

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

void ThicknessGauge::captureFrames() {
	if (!cap.isOpened()) // check if we succeeded
		throw CaptureFailException("Error while attempting to open capture device.");

	cv::Mat t;
	for (auto i = 0; i++ < frameCount_;) {
		cap >> t;
		frames.push_back(t);
	}

	setImageSize(t.size());

}

void ThicknessGauge::laplace(cv::Mat& image) const {
	cv::Mat tmp;
	Laplacian(image, tmp, settings.ddepth, settings.kernelSize); // , scale, delta, BORDER_DEFAULT);
	convertScaleAbs(tmp, image);
}

void ThicknessGauge::sobel(cv::Mat& image) const {
	Sobel(image, image, -1, 1, 1, settings.kernelSize, settings.scale, settings.delta, cv::BORDER_DEFAULT);
}

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

	std::cout << "Frames split without crash :-)" << endl;

}

template <int minLen>
int ThicknessGauge::computeHoughPMinLine(cv::Rect2f& rect) const {
	auto minLineLen = cvRound(rect.width / 32);

	if (minLineLen < minLen)
		minLineLen = minLen;

	return minLineLen;
}

void ThicknessGauge::computeLaserLocations(shared_ptr<LaserR> laser, cv::Vec4f& baseLine, shared_ptr<FilterR> filter, cv::Rect2f& markingLocation, std::vector<cv::Point2f>& result) {

	// generate frames with marking
	std::vector<cv::Mat> markingFrames;
	std::vector<cv::Mat> outputs;
	std::vector<cv::Point2f> pixPlanar;
	std::vector<cv::Point2f> test_subPix;
	std::vector<cv::Point> nonZeroes;

	pixPlanar.reserve(frameCount_);
	test_subPix.reserve(frameCount_);

	for (auto& frame : frames) {
		markingFrames.push_back(frame(markingLocation));
		outputs.push_back(cv::Mat::zeros(markingFrames.back().size(), CV_8UC1));
	}

	const std::string windowName = "test height";
	draw->makeWindow(windowName);

	showWindows_ = true;

	cv::Mat tmpOut;

	auto highestPixelTotal = 0.0;

	std::vector<cv::Point2d> tmp;

	while (true) {

		auto start = cv::getTickCount();

		auto highestPixel = 0.0;

		cv::Mat target; // the laser is herby contained!!!

		for (auto i = frameCount_; i--;) {

			cv::Mat baseFrame;
			outputs[i] = cv::Mat::zeros(markingFrames.back().size(), CV_8UC1);

			std::vector<cv::Point> nonZero(outputs.at(i).rows * outputs.at(i).cols);

			// TODO : replace with custom filter if needed
			cv::bilateralFilter(markingFrames.at(i), baseFrame, 3, 20, 10);

			threshold(baseFrame, baseFrame, 100, 255, CV_THRESH_BINARY);

			GaussianBlur(baseFrame, baseFrame, cv::Size(5, 5), 0, 10, cv::BORDER_DEFAULT);

			/* RECT CUT METHOD */
			findNonZero(baseFrame, nonZero);
			cv::Rect laserArea = cv::boundingRect(nonZero);
			cv::Mat t = baseFrame(laserArea);
			highestPixel += LineCalc::computeRealIntensityLine(t, tmp, t.rows, 0, "_marking", laserArea.y);
			highestPixel += (laserArea.y);

			/* FULL COLUMN METHOD */
			//highestPixel += LineCalc::computeRealIntensityLine(baseFrame, tmp, baseFrame.rows, 0, "_marking");

			/* OBSOLUTE? METHOD */
			//auto generateOk = miniCalc.generatePlanarPixels(baseFrame, outputs.at(i), pixPlanar, test_subPix);

			//if (!generateOk) {
			//	Util::loge("Error while attempting to generate pixelmap");
			//	continue;
			//}

			//nonZeroes.reserve(baseFrame.cols * baseFrame.rows);
			//findNonZero(outputs.at(i), nonZeroes);
			//auto heightLine = baseFrame.rows / 2;
			//highestPixel += static_cast<float>(baseFrame.rows) - static_cast<float>(pix.getHighestYpixel(outputs.at(i), heightLine, miniCalc));
			if (draw->isEscapePressed(30)) {
				showWindows_ = false;
				break;
			}

			if (!i)
				cv::cvtColor(outputs.at(i), tmpOut, CV_GRAY2BGR);
		}

		highestPixelTotal = frames.front().rows - (highestPixel / static_cast<unsigned int>(frameCount_));
		auto end = cv::getTickCount();
		std::cout << "highestPixelTotal: " << highestPixelTotal << endl;

		auto diff = baseLine[1] - highestPixelTotal;
		std::cout << "diff: " << diff << endl;

		auto time = (end - start) / cv::getTickFrequency();
		std::cout << "time : " << time << endl;
		if (showWindows_) {
			draw->drawHorizontalLine(&tmpOut, cvRound(highestPixelTotal), cv::Scalar(0, 255, 0));
			//draw->drawHorizontalLine(&tmpOut, cvRound(diff), cv::Scalar(0, 0, 255));
			draw->drawHorizontalLine(&tmpOut, cvRound(frames.front().rows - baseLine[1]), cv::Scalar(0, 0, 255));
			draw->drawText(&tmpOut, to_string(diff) + " pixels", TextDrawPosition::UpperLeft);
			draw->drawText(&tmpOut, to_string(time) + "s", TextDrawPosition::UpperRight);
			draw->showImage(windowName, tmpOut);
			if (draw->isEscapePressed(30))
				showWindows_ ^= true;
		}

		if (!showWindows_)
			break;

	}

	draw->removeWindow(windowName);

}

/**
 * \brief Main entry points for calculation of marking height
 */
void ThicknessGauge::computeMarkingHeight(std::string& globName) {

	// determin where to get the frames from.
	if (globName == "camera")
		captureFrames();
	else
		loadGlob(globName);

	// configure frames based on center vertical splitting of the original frames
	vector<cv::Mat> leftFrames(frameCount_);
	vector<cv::Mat> rightFrames(frameCount_);

	splitFrames(leftFrames, rightFrames);

	// morph extension class for easy use
	auto morph = make_shared<MorphR>(cv::MORPH_GRADIENT, 1, showWindows_);

	// common canny with default settings for detecting marking borders
	auto canny = make_shared<CannyR>(200, 250, 3, true, showWindows_, true);

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
	cv::Rect2f markingRect;

	auto markingOk = computerMarkingRectangle(canny, markingFilter, houghV, markingRect);

	if (!markingOk)
		Util::loge("Error while computing marking rectangle.");

	// make sure the minimum is at least 10 pixels.
	auto minLineLen = computeHoughPMinLine<10>(markingRect);

	// horizontal houghline class
	auto houghH = make_shared<HoughLinesPR>(1, cvRound(CV_PI / 180), 40, minLineLen, true);

	houghH->setMaxLineGab(12);
	houghH->setMarkingRect(markingRect);

	// the baselines, which are located outside the marking
	cv::Vec4f baseLines;

	computeBaseLineAreas(canny, baselineFilter, houghH, morph, baseLines);

	std::cout << "left  base line Y : " << baseLines[1] << std::endl;
	std::cout << "right base line Y : " << baseLines[3] << std::endl;

	// the locations for where the base lines intersect with the marking border
	cv::Vec4f intersections;

	LineCalc lineCalc;

	// compute the intersection points based on the borders of the markings and the baseline for the laser outside the marking
	lineCalc.computeIntersectionPoints(baseLines, houghV->getLeftBorder(), houghV->getRightBorder(), intersections);

	std::cout << "intersection points: " << intersections << std::endl;

	// TODO : figure out a clever way to calculate this..
	const uint pixelCutoff = 40; // must never be higher than marking rect width

	// adjust the marking rect according to the intersection points
	lineCalc.adjustMarkingRect(markingRect, intersections, pixelCutoff);

	// adjust the baselines according to the intersection points. (could perhaps be useful in the future)
	lineCalc.adjustBaseLines(baseLines, intersections, pixelCutoff);

	std::cout << "adjusted marking rect: " << markingRect << std::endl;
	std::cout << "adjusted left  base line Y : " << baseLines[1] << std::endl;
	std::cout << "adjusted right base line Y : " << baseLines[3] << std::endl;

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

void ThicknessGauge::computeBaseLineAreas(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, shared_ptr<MorphR> morph, cv::Vec4f& output) {

	LineLaserData results;

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

	int bias = Left;

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

	SparseR<float> sparse;

	auto imSize = left.front().size();
	auto imType = left.front().type();

	auto lineY = 0.0f;
	unsigned int totalY = 0;

	auto avgTotal = 0.0;
	std::vector<cv::Point2d> tmp;

	std::vector<cv::Point2f> allElements;


#define _sparse_mode

	while (true) {

		auto avg = 0.0;

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
			for (auto& h : lines) {
				//totalY += 2;
				//auto t = (h.entry[1] + h.entry[3]) * 0.5f;
				//lineY += t;
				Util::copyVector(h.elements, allElements);
			}

			if (draw->isEscapePressed(30)) {
				showWindows_ ^= true;
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
			if (draw->isEscapePressed(30)) {
				showWindows_ ^= true;
				break; // escape was pressed
			}
		}

		/*lineY /= totalY;*/

		boundryRect.width -= 40;

		cv::Mat t = org(boundryRect);
		lineY = frames.front().rows - quarter + boundryRect.y + LineCalc::computeRealIntensityLine(t, tmp, t.rows, 0, "_left_baseline", frames.front().rows - quarter + boundryRect.y);
		//cout << "baseline new: " << lineY << endl;
		//cout << "baseline org: " << lineY + (frames.front().rows - baseLineY) << endl;

		if (!showWindows_)
			break;

		lineY = 0.0f;
		//totalY = 0;

	}

	output[0] = 0.0f;
	output[1] = lineY;
	//output[1] = lineY + (frames.front().rows - baseLineY);

	// TODO : Add right side line computation
	// just clone left to right for now
	output[2] = output[0];
	output[3] = output[1];

	draw->removeWindow(leftWindow);
	draw->removeWindow(rightWindow);
}

bool ThicknessGauge::computerMarkingRectangle(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesR> hough, cv::Rect2f& output) {

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

	const std::string windowName = "test marking out";
	draw->makeWindow(windowName);

	vector<cv::Rect2f> markingRects;

	while (true) {

		cv::Mat markingTest;
		markingRects.reserve(frameCount_);
		cv::Mat sparse;
		for (auto i = frames.size(); i--;) {
			markingTest = frames.at(i).clone();
			auto org = frames.at(i).clone();
			filter->setImage(frames.at(i).clone());
			filter->doFilter();
			canny->setImage(filter->getResult());
			canny->doCanny();

			cv::Mat t = canny->getResult();

			hough->setOriginal(t);
			hough->setImage(t);

			hough->doVerticalHough();
			hough->computeBorders();
			markingRects.push_back(hough->getMarkingRect());
			if (draw->isEscapePressed(30))
				return true;
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
		cout << "Final marking rect : " << output << endl;
		if (!showWindows_)
			break;

		draw->drawRectangle(markingTest, output, cv::Scalar(128, 128, 128));
		draw->showImage(windowName, markingTest);
	}

	draw->removeWindow(windowName);

	return output.x > 0.0f && output.y > 0.0f && output.width > 0.0f;
}

/**
 * \brief Determins the marking boundries
 * \param globName if "camera", use camera, otherwise load from glob folder
 * \return 2 Float vector with the points marking the boundries as pair, where first = left, second = right
 */
LineBaseData ThicknessGauge::findMarkingLinePairs_(std::string& globName) {

	array<cv::Mat, 512> outputs;

	linePair result(cv::Point2i(0, 0), cv::Point2i(0, 0));

	CannyR cannyH(200, 250, 3, false, showWindows_, true);
	CannyR cannyV(10, 30, 3, true, false, false);

	HoughLinesR houghV(1, CV_PI / 180, 100, showWindows_);
	HoughLinesPR houghP(1, CV_PI / 180, 100, 20, showWindows_);
	FilterR lineFilter("LineFilterH");
	FilterR lineFilterV("LineFilterV");

	//FilterR speckFilter("SpeckFilter");

	cv::Mat lineHKernel = (cv::Mat_<char>(1, 5) << 3 , 3 , 0 , -3 , -3);
	cv::Mat speckKernel = (cv::Mat_<char>(3, 3) <<
		0 , 0 , 0 ,
		0 , 1 , 0 ,
		0 , 0 , 0);

	cv::Mat lineVKernel = (cv::Mat_<char>(4, 4) <<
		0 , 0 , 1 , 1 ,
		0 , 1 , 1 , 1 ,
		1 , 1 , 1 , 0 ,
		1 , 1 , 0 , 0
	);

	lineFilter.setKernel(lineHKernel);
	lineFilter.setKernel(lineVKernel);
	//speckFilter.setKernel(speckKernel);

	//filter.generateKernel(6, 6, 1.0f);

	houghV.setAngleLimit(30);
	houghP.setAngleLimit(30);

	Pixelz pixelz;

	LineBaseData results;

	while (true) {

		uint64 time_begin = cv::getTickCount();

		vector<double> baseLine(frameCount_);

		// capture frame amount and clear storage
		for (auto i = frameCount_; i--;) {
			outputs[i] = cv::Mat::zeros(imageSize_, CV_8UC1);
		}

		//HistoPeak hp;

		for (auto i = 0; i < frames.size(); ++i) {

			// just share the joy
			auto frame = frames.at(i).clone();

			cv::Mat hori = frames.at(i).clone();
			cv::Mat vert = frames.at(i).clone();

			cv::Mat tmp = frames.at(i).clone();

			//cv::bilateralFilter(frame, tmp, 1, 20, 10);

			//speckFilter.setOriginal(frames.at(i));
			//speckFilter.setImage(tmp);
			//speckFilter.doFilter();
			//tmp = speckFilter.getResult();

			lineFilter.setOriginal(frame);
			lineFilter.setImage(hori);
			lineFilter.doFilter();
			//tmp = lineFilter.getResult();

			lineFilterV.setOriginal(frame);
			lineFilterV.setImage(vert);
			lineFilterV.doFilter();
			tmp = lineFilterV.getResult();

			//cv::threshold(tmp, tmp, 200, 255, cv::THRESH_BINARY);

			cannyH.setImage(tmp);
			cannyH.doCanny();
			tmp = cannyH.getResult();

			houghP.setOriginal(frame);
			houghP.setImage(tmp);
			houghP.doHorizontalHough();

			//cannyV.setImage(frame);
			//cannyV.doCanny();

			houghV.setOriginal(frames.at(i));
			houghV.setImage(tmp.clone());
			houghV.doVerticalHough();

			// show default input image
			if (showWindows_) {
				imshow(windowMainTitle, frame);
			}

			if (showWindows_) {
				//Line l;
				auto num(to_string(i));
				//l.setFrame(frame);
				//l.generateSparse();
				//l.differentiateY();
				//l.differentiateIntensity();
				//l.mergeIntensity();
				//l.saveAllData(num);
				//l.drawPoly();

				//Histogram g;
				//g.populateHistogram(frame);
				//cv::imshow(line2WindowName, g.histogramImage());
				//auto filename("test_histo_" + num + ".txt");
				//g.saveSimpleData(filename);

				//auto blobs = drawBlobs(&frame);
				//imshow("keypoints", blobs);

			}

			if (draw->isEscapePressed(10))
				return LineBaseData(); // esc
		}
	}
}

bool ThicknessGauge::savePlanarImageData(string filename, vector<cv::Point>& pixels, cv::Mat& image, double highestY, string timeString, string extraInfo) const {
	cv::FileStorage fs(filename + ".json", cv::FileStorage::WRITE);

	if (!fs.isOpened()) {
		cerr << "Error while opening " << filename << " for output." << endl;
		return false;
	}

	ostringstream oss;
	oss << Util::getTime();
	cout << "Saving data [frames: " << frameCount_ << "] [time:" << oss.str() << "] [pixels:" << pixels.size() << ']' << endl;

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
		cout << "sum [row] : " << row << "-> " << sum << "\n";
	}
}

void ThicknessGauge::computerGaugeLine(cv::Mat& output) {
	vi aboveLine;

	if (miniCalc.getActualPixels(allPixels_, aboveLine, baseLine_[0], output.rows)) {
		//cout << "Retrived " << aboveLine.size() << " elements above line.\n";
		if (miniCalc.computerCompleteLine(aboveLine, gaugeLine_, lineConfig_)) {
			//cout << "Computed line fitting... " << gaugeLine_ << "\n";

			gaugeLineSet_ = true;

			// sort the elements for quick access to first and last (outer points in line)
			sort(aboveLine.begin(), aboveLine.end(), miniCalc.sortX);

			avgGaugeHeight_ = gaugeLine_[3];

			if (showWindows_) {
				line(output, cv::Point2f(static_cast<float>(aboveLine.front().x) + gaugeLine_[0], gaugeLine_[3]), cv::Point2f(static_cast<float>(aboveLine.back().x), gaugeLine_[3]), baseColour_, 2, cv::LINE_AA);

				//cout << "Average line height : " << output.rows - avgGaugeHeight_ << " elements.\n";
			}
		}
		else {
			gaugeLineSet_ = false;
			Util::loge("Failed to generate fitted line.");
		}
	}
	else {
		gaugeLineSet_ = false;
		Util::loge("Failed to retrive elements above line.");
	}

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
