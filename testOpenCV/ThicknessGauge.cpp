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

	for (auto i = 0; i < size; ++i)
		frames[i] = cv::imread(files.at(i), CV_8UC1);

	setImageSize(frames[0].size());

}

void ThicknessGauge::captureFrames() {
	if (!cap.isOpened()) // check if we succeeded
		throw CaptureFailException("Error while attempting to open capture device.");

	cv::Mat t;
	for (auto i = 0; i++ < frameCount_;) {
		cap >> t;
		frames[i] = t;
	}

	setImageSize(t.size());

}

void ThicknessGauge::Blur(cv::Mat& image, cv::Size size) {
	GaussianBlur(image, image, size, 1.5, 1.5);
}

void ThicknessGauge::MeanReduction(cv::Mat& image) {
	MeanReduction(image);
}

void ThicknessGauge::laplace(cv::Mat& image) const {
	cv::Mat tmp;
	Laplacian(image, tmp, settings.ddepth, settings.kernelSize); // , scale, delta, BORDER_DEFAULT);
	convertScaleAbs(tmp, image);
}

void ThicknessGauge::sobel(cv::Mat& image) const {
	Sobel(image, image, -1, 1, 1, settings.kernelSize, settings.scale, settings.delta, cv::BORDER_DEFAULT);
}

void ThicknessGauge::drawPlarnarPixels(cv::Mat& targetImage, vector<cv::Point>& planarMap) const {
	polylines(targetImage, planarMap, false, cv::Scalar(255, 255, 255), 2);
}

inline void ThicknessGauge::computeAllElements(cv::Mat& image) {
	findNonZero(image, allPixels_);
}

double ThicknessGauge::computerBaseLine(const cv::Mat& image, double limit) {

	cv::Mat dst, cdst;
	Canny(image, dst, 20, 100, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	typedef pair<cv::Point, cv::Point> Points;

	vector<cv::Vec2f> hlines;
	vector<Points> allHLines;
	// detect lines
	HoughLines(dst, hlines, 1, CV_PI / 180, lineThreshold_);

	auto baseLineAvg = 0.0;
	auto count = 0;

	cv::Rect roi(0, image.rows - (image.rows / 2), image.cols, image.rows / 2);

	//cout << "roi : " << roi << endl;

	//double limit = image.rows / 2;

	// adjust limit
	//limit += image.rows / 2;

	auto bestGuess = 0.0;

	for (auto& l : hlines) {
		auto theta = l[1];
		if (theta > CV_PI / 180 * 89.99 && theta < CV_PI / 180 * 90.01) {
			auto rho = l[0];
			auto a = cos(theta);
			auto b = sin(theta);
			auto x0 = a * rho;
			auto y0 = b * rho;
			cv::Point pt1(cvRound(x0 + 1000 * (-b)),
			              cvRound(y0 + 1000 * (a)));
			cv::Point pt2(cvRound(x0 - 1000 * (-b)),
			              cvRound(y0 - 1000 * (a)));

			//if (roi.contains(pt1) && roi.contains(pt2)) {
			if (pt1.y > limit && pt2.y > limit) {
				//cout << "pt1.y : " << pt1.y << endl;
				double yAvg = (pt1.y + pt2.y) / 2;
				// check for best guess
				if (yAvg > bestGuess)
					bestGuess = yAvg;

				count += 2;
				allHLines.push_back(Points(pt1, pt2));
				baseLineAvg += pt1.y + pt2.y;
				if (showWindows_) {
					//line(image, Point(0, bestGuess), Point(image.cols, bestGuess), baseColour_);
					//line(image, pt1, pt2, baseColour_, 1);
				}
			}
		}
	}

	//cout << "new baseline : " << (image.rows) - baseLineAvg / count << endl;

	return bestGuess;
	//return (image.rows) - baseLineAvg / count;
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

bool ThicknessGauge::generatePlanarImage(std::string& globName) {
	//if (!cap.isOpened() && globName == "camera") // check if we succeeded
	//	throw CaptureFailException("Error while attempting to open capture device.");

	cv::Size blurSize(3, 3);

	const auto alpha = 0.5;
	const auto beta = 1.0 - alpha;

	auto line_fraction = 0;
	auto line_thickness = 1;

	/* erosion and dilation trackbar settings */
	auto erosion_type = 2;
	auto erosion_size = 3;

	auto dilation_type = 0;
	auto dilation_size = 1;

	auto const max_ed_elem = 2;
	auto const max_ed_kernel_size = 21;
	/* end */

	ImageSave is("pic_x", SaveType::Image_Png, Information::Basic);

	cv::Mat frame;
	//vector<Mat> outputs(frameCount_);
	//vector<vi> pix_Planarmap(frameCount_ * 2); // using double of these for testing
	vector<v2<double>> gabs(frameCount_);

	array<cv::Mat, 512> outputs;
	array<vi, 512> pix_Planarmap;
	//array<v2<int>, 512> gabs;

	vi nonZero;

	// capture first frame
	if (globName == "camera")
		captureFrames();
	else
		loadGlob(globName);

	auto heightLine = imageSize_.width;

	const string inputWindowName = "GC2450 feed";
	const string outputWindowName = "GC2450 manipulated";
	const string line1WindowName = "frame";
	const string line2WindowName = "sparse y";
	const string line3WindowName = "GC2450 weighted means over time";
	const string cornerWindowName = "GC2450 Corner View";
	const string erodeWindowName = "Erosion";
	const string dilationWindowName = "Dilation";

	if (showWindows_) {
		namedWindow(inputWindowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("BThreshold", inputWindowName, &binaryThreshold_, 254);
		cv::createTrackbar("HThreshold", inputWindowName, &lineThreshold_, 255);
		//createTrackbar("Base Line", inputWindowName, &baseLine_, imageSize_.height);
		cv::createTrackbar("Height Line", inputWindowName, &heightLine, (imageSize_.width * 2) - 1);

		namedWindow(outputWindowName, cv::WINDOW_KEEPRATIO);

		//namedWindow(line1WindowName, cv::WINDOW_KEEPRATIO);

		////createTrackbar("Frac", line1WindowName, &line_fraction, 4);
		////createTrackbar("Thick", line1WindowName, &line_thickness, 5);

		//namedWindow(line2WindowName, cv::WINDOW_KEEPRATIO);


		//namedWindow(line3WindowName, WINDOW_AUTOSIZE);

		//namedWindow(cornerWindowName, WINDOW_AUTOSIZE);

		namedWindow(erodeWindowName, cv::WINDOW_KEEPRATIO);
		//createTrackbar("Element:", erodeWindowName, &erosion_type, max_ed_elem);
		//createTrackbar("Kernel size: 2n +1", erodeWindowName, &erosion_size, max_ed_kernel_size);

		//namedWindow(dilationWindowName, WINDOW_AUTOSIZE);
		//createTrackbar("Element:", dilationWindowName, &dilation_type, max_ed_elem);
		//createTrackbar("Kernel size: 2n +1", dilationWindowName, &dilation_size, max_ed_kernel_size);
	}

	// test for video recording
	if (saveVideo_) {
		is.SetInformation(Information::Full);
		is.SetSaveType(SaveType::Video);
		is.SetCodec(VideoCodec::Mjpeg);
		is.SetFPS(25.0f);
		is.SetSize(frame.cols, frame.rows);
		is.SetColour(VideoColour::Colour);
		is.SetFileName("_testvideo");
		is.OpenVideo();
	}

	vector<cv::Point2d> test_subPix;

	// configure output stuff
	for (auto i = frameCount_; i--;)
		pix_Planarmap[i].reserve(imageSize_.width);

	for (auto i = arrayLimit - 1; i > arrayLimit - frameCount_; i--)
		pix_Planarmap[i].reserve(imageSize_.width);

	// start the process of gathering information for set frame count
	while (true) {

		CannyR canny(100, 150, 3, false, showWindows_);
		HoughLinesR houghL(1, CV_PI / 180, 100, showWindows_);

		uint64 time_begin = cv::getTickCount();

		vector<double> baseLine(frameCount_);

		// capture frame amount and clear storage
		for (auto i = 0; i < frameCount_; ++i) {

			//cap >> frame;
			//frames[i] = cv::Mat::zeros(imageSize_, CV_8UC1);
			//outputs[i] = cv::Mat::zeros(imageSize_, CV_8UC1);

			//for (auto& n : nulls_)
			//	frames[i] = frame - n;

			//pix_Planarmap.at(i).clear();

			//cap >> frames[i];
			outputs[i] = cv::Mat::zeros(imageSize_, CV_8UC1);
			pix_Planarmap.at(i).clear();
		}

		//HistoPeak hp;

		for (auto i = 0; i < frameCount_; ++i) {

			// just share the joy
			frame = frames[i];

			cv::Mat tmp;
			cv::bilateralFilter(frame, tmp, 1, 20, 10);

			canny.setImage(tmp);
			canny.doCanny();

			houghL.setOriginal(tmp);
			houghL.setImage(canny.getResult());
			houghL.setAngleLimit(20);
			houghL.doVerticalHough();
			//houghL.doHorizontalHough();

			// show default input image (always shown live!)
			if (showWindows_) {
				imshow(inputWindowName, tmp);
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
			if (showWindows_) {
				//hp.processImage(frame, true, i != 0);

			}

			//equalizeHist(frame, frame);

			// do basic in-place binary threshold
			//threshold(frame, frame, binaryThreshold_, 255, CV_THRESH_BINARY);

			// blur in-place
			//GaussianBlur(frame, frame, cv::Size(7, 5), 10, 0, cv::BORDER_DEFAULT);
			//GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);


			// perform some stuff
			// laplace(frame);
			// c.Sobel(frame);

			if (showWindows_) {
				imshow(outputWindowName, tmp);
			}
			// extract information from the image, and make new output based on pixel intensity mean in Y-axis for each X point
			auto generateOk = miniCalc.generatePlanarPixels(tmp, outputs[i], pix_Planarmap.at(i), test_subPix);

			if (!generateOk) {
				cout << "Failed to map pixels to 2D plane for frame #" << to_string(i + 1) << " of " << to_string(frameCount_) << endl;
				continue;
			}

			findNonZero(outputs[i], pix_Planarmap[arrayLimit - (1 + i)]);
			//gabs.push_back(miniCalc.fillElementGabs(pix_Planarmap[arrayLimit - (1 + i)], outputs[i], baseLine_));
			gabs.push_back(miniCalc.fillElementGabs(pix_Planarmap[arrayLimit - (1 + i)], outputs[i]));
			if (gabs.back().hasValue()) {
				// at least one gab was filled..

				//Mat temp(pix_planarMap[arrayLimit - (1 + i)]);
				//cout << "TEMP cols " << temp.cols << endl;
				//imshow("temp", temp);
				//add(outputs[i], temp, outputs[i]);
			}

			auto highestPixel = outputs[i].rows - pix.getHighestYpixel(outputs[i], heightLine, miniCalc);

			auto bl = computerBaseLine(outputs[i], highestPixel);
			if (cvIsNaN(bl)) {
				cerr << "Error while computing baseline for frame " << to_string(i) << '\n';
				continue;
			}

			baseLine.push_back(bl);

			if (showWindows_) {
				auto key = static_cast<char>(cv::waitKey(10));
				if (key == 27)
					return true; // esc
			}


		}

		houghL.alignLeftY(frameCount_);


		//cout << hp << endl;

		baseLine_[0] = miniCalc.mean(baseLine);
		//cout << "baseline real : " << baseLine_ << endl;

		frame = cv::Mat::zeros(imageSize_, CV_8UC1);
		cv::Mat lines = cv::Mat::zeros(imageSize_, CV_8UC1);

		// merge the images to target
		for (auto i = 0; i < frameCount_; ++i) {
			addWeighted(outputs[i], alpha, lines, beta, 0.0, lines);
			//add(outputs[i], lines, lines);
			outputs[i].release();
			if (saveVideo_) is.SaveVideoFrame(lines);
		}


		cv::Mat output = cv::Mat::zeros(imageSize_, lines.type());

		//if (showWindows_) imshow(line2WindowName, output);
		//if (showWindows_) imshow(line3WindowName, lines);

		//is.UpdateTimeStamp();
		//is.SaveImage(&lines);

		//auto corner_image = cornerHarris_test(lines, 200);
		//if (showWindows_) imshow(cornerWindowName, corner_image);

		/* test stuff for filtering out crap pixels */
		addWeighted(lines, 1.5, lines, -0.5, 0, output);

		//skeleton(&output);
		auto erosion_image = this->erosion(output, erosion_type, erosion_size);
		bilateralFilter(erosion_image, output, 1, 20, 10);
		//bilateralFilter(erosion_image, output, 1, 80, 20);
		erosion_image.release();

		/* end test stuff */

		resize(output, frame, output.size() * 2, 0, 0, cv::INTER_LANCZOS4);

		GaussianBlur(frame, output, blurSize, 10, 10, cv::BORDER_CONSTANT);

		frame.release();

		//resize(output, frame, frame.size() / 2, 0, 0, INTER_LANCZOS4);

		// test for highest pixel for eroded image
		auto highestPixel = output.rows - pix.getHighestYpixel(output, heightLine, miniCalc) - baseLine_[0];
		cout << "Highest Y in eroded line : " << highestPixel << " [mm: N/A ]" << endl;

		/* base line etc calculations !!*/

		// gather all elements from final matrix
		computeAllElements(output);

		computerGaugeLine(output);

		frameTime_ = cv::getTickCount() - time_begin;

		if (showWindows_) {
			drawVerticalLine(&output, heightLine);
			// calculated baseline test drawing
			drawHorizontalLine(&output, Util::round(baseLine_[0]));
		}

		if (showWindows_) {
			imshow(erodeWindowName, output);
		}


		//cout << "Y avr for heightline : " << getYPixelsAvg(frame, heightLine) << endl;

		//cout << "Saving image...\n";
		//is.UpdateTimeStamp();
		//is.SaveImage(&output, "_testoutput" + to_string(frameCount_));
		//savePlanarImageData("_testoutput", allPixels_, output, highestPixel);

		if (showWindows_) {
			auto key = static_cast<char>(cv::waitKey(10));
			if (key == 27)
				break; // esc
		}

		output.release();

	}

	cap.release();

	if (showWindows_) {
		cv::destroyWindow(inputWindowName);
		cv::destroyWindow(outputWindowName);
		cv::destroyWindow(line1WindowName);
		cv::destroyWindow(line2WindowName);
		cv::destroyWindow(line3WindowName);
	}

	if (saveVideo_)
		is.CloseVideo();

	return true;
}


void ThicknessGauge::splitFrames(vector<cv::Mat>& left, vector<cv::Mat>& right) {

	if (!left.empty())
		left.clear();

	left.reserve(frameCount_);

	if (!right.empty())
		right.clear();

	right.reserve(frameCount_);

	cv::Point topLeft(0, 0);
	cv::Point buttomLeft(frames[0].cols / 2, frames[0].rows);

	cv::Point topRight(frames[0].cols / 2, 0);
	cv::Point buttomRight(frames[0].cols, frames[0].rows);

	cv::Rect leftRect2I(topLeft, buttomLeft);
	cv::Rect rightRect2I(topRight, buttomRight);

	for (auto i = frameCount_; i--;) {
		left.push_back(frames[i](leftRect2I));
		right.push_back(frames[i](rightRect2I));
	}

	cout << "Frames split without crash :-)" << endl;

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

	vector<cv::Mat> leftFrames;
	vector<cv::Mat> rightFrames;

	splitFrames(leftFrames, rightFrames);

	// common canny with default settings for detecting marking borders
	auto canny = make_shared<CannyR>(200, 250, 3, true, showWindows_);
	auto markingFilter = make_shared<FilterR>("MarkingFilter");
	auto baselineFilter = make_shared<FilterR>("Baseline Filter");
	auto houghV = make_shared<HoughLinesR>(1, static_cast<const int>(CV_PI / 180), 40, showWindows_);
	houghV->setAngleLimit(30);

	markingFilter->setShowWindow(showWindows_);
	baselineFilter->setShowWindow(showWindows_);

	cv::Rect2f markingRect;

	auto markingOk = computerMarkingRectangle(canny, markingFilter, houghV, markingRect);

	auto houghH = make_shared<HoughLinesPR>(1, static_cast<const int>(CV_PI / 180), 40, true);

	houghH->setMaxLineGab(12);
	houghH->setMinLineLen(cvRound(markingRect.width / 2));
	houghH->setMarkingRect(markingRect);

	std::array<cv::Rect2f, 2> baseLines;

	computeBaseLineAreas(canny, baselineFilter, houghH, baseLines);

	if (showWindows_) {
		auto key = static_cast<char>(cv::waitKey(30));
		if (key == 27)
			return; // escape was pressed
	}


}

LineLaserData ThicknessGauge::computeBaseLineAreas(shared_ptr<CannyR> canny, shared_ptr<FilterR> filter, shared_ptr<HoughLinesPR> hough, std::array<cv::Rect2f, 2>& output) {

	LineLaserData results;

	cv::Mat lineHKernel = (cv::Mat_<char>(4, 1) <<
		0 ,
		1 ,
		1 ,
		0
	);

	filter->setKernel(lineHKernel);

	cv::namedWindow("test baseline left", CV_WINDOW_FREERATIO);
	cv::namedWindow("test baseline right", CV_WINDOW_FREERATIO);

	int bias = Left;

	auto quarter = static_cast<float>(frames[0].rows / 4);
	auto baseLineY = frames[0].rows - quarter;

	cv::Rect2f leftBaseLine;
	leftBaseLine.x = 0.0f;
	leftBaseLine.y = baseLineY;
	leftBaseLine.width = hough->getMarkingRect().x;
	leftBaseLine.height = quarter;

	cv::Rect2f rightBaseLine;
	rightBaseLine.x = leftBaseLine.width + hough->getMarkingRect().width;
	rightBaseLine.y = baseLineY;
	rightBaseLine.width = frames[0].cols - rightBaseLine.x;
	rightBaseLine.height = leftBaseLine.height;

	while (true) {

		for (auto i = frameCount_; i--;) {
			cv::Mat left = frames[i](leftBaseLine);
			cv::Mat right = frames[i](rightBaseLine);
			if (showWindows_) {
				cv::imshow("test baseline left", left);
				cv::imshow("test baseline right", right);
				auto key = static_cast<char>(cv::waitKey(30));
				if (key == 27) {
					showWindows_ ^= true;
					break; // escape was pressed
				}
			}


		}


		if (!showWindows_)
			break;
	}


	return LineLaserData();


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

	cv::namedWindow("test marking out", CV_WINDOW_FREERATIO);

	vector<cv::Rect2f> markingRects;

	while (true) {

		markingRects.reserve(frameCount_);

		for (auto i = frameCount_; i--;) {
			cv::Mat markingTest = frames[i].clone();
			cv::Mat org = frames[i].clone();
			filter->setImage(frames[i].clone());
			filter->doFilter();
			canny->setImage(filter->getResult());
			canny->doCanny();
			hough->setOriginal(org);
			hough->setImage(canny->getResult());
			hough->doVerticalHough();
			hough->computeBorders();
			markingRects.push_back(hough->getMarkingRect());
			if (showWindows_) {
				cv::rectangle(markingTest, output, cv::Scalar(128, 128, 128), 2, cv::LINE_AA);
				cv::imshow("test marking out", markingTest);
				auto key = static_cast<char>(cv::waitKey(30));
				if (key == 27)
					return true; // escape was pressed
			}
		}
		// calculate the avg rect
		output.x = 0.0f;
		output.y = 0.0f;
		output.width = 0.0f;
		output.height = frames[0].rows;
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
	}

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

	CannyR cannyH(200, 250, 3, false, showWindows_);
	CannyR cannyV(10, 30, 3, true, false);

	HoughLinesR houghV(1, CV_PI / 180, 100, showWindows_);
	HoughLinesPR houghP(1, CV_PI / 180, 100, showWindows_);
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

		for (auto i = 0; i < frameCount_; ++i) {

			// just share the joy
			auto frame = frames[i].clone();

			cv::Mat hori = frames[i].clone();
			cv::Mat vert = frames[i].clone();

			cv::Mat tmp = frames[i].clone();

			//cv::bilateralFilter(frame, tmp, 1, 20, 10);

			//speckFilter.setOriginal(frames[i]);
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

			houghV.setOriginal(frames[i]);
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

			if (showWindows_) {
				auto key = static_cast<char>(cv::waitKey(10));
				if (key == 27)
					return LineBaseData(); // esc
			}
		}
	}
}

void ThicknessGauge::addKernelTests(vector<TestConfig>& tests, float alpha, int baseSigmaX, int x, int y) {
	for (auto j = x; j <= y; j += 2) {
		for (auto i = 1; i <= 10; ++i) {
			if (x == j)
				continue;
			auto sig = baseSigmaX * i;
			tests.push_back(TestConfig(alpha, sig, i, cv::Size(x, x)));
			tests.push_back(TestConfig(alpha, sig, i, cv::Size(x, j)));
			tests.push_back(TestConfig(alpha, sig, i, cv::Size(j, x)));
		}
	}
}

bool ThicknessGauge::testDiff() {

	if (!cap.isOpened()) // check if we succeeded
		throw CaptureFailException("Error while attempting to open capture device.");

	Util::log("Initiating test mode.. please wait...");

	ImageSave is("test_x", SaveType::Image_Png, Information::Basic);

	// kernel size vector
	const vector<cv::Size> kernels_ = {cv::Size(0, 0), cv::Size(3, 3), cv::Size(5, 5), cv::Size(7, 7), cv::Size(9, 9)};

	// weigthed adding boundries for alpha
	// beta values are always 1.0 = alpha
	float alphaBase = 0.1f;

	// blur sigma boundries
	auto sigmaXBase = 5;

	// each test is put here, to better control the flow
	vector<TestConfig> tests;

	const auto arrayLimit = 1024; // shit c++11 ->

	cv::Mat frame;
	vector<v2<double>> gabs(frameCount_);

	array<cv::Mat, arrayLimit> frames;
	array<cv::Mat, arrayLimit> outputs;
	array<vi, arrayLimit> pix_Planarmap;

	array<vi, arrayLimit> sparse;

	vi nonZero;

	cv::Mat first;
	// capture first frame, only to get sizes etc.
	cap >> first;

	auto out("Capturing " + to_string(frameCount_) + " frames..");

	//ProgressBar progress(frameCount_, out.c_str());
	//progress.SetFrequencyUpdate(10);
	//progress.SetStyle(">", "-");

	for (auto i = frameCount_; i--;) {
		//progress.Progressed(i);
		cap >> frames[i];
	}
	//progress.Progressed(frameCount_);

	Util::log("");

	cap.release();

	setImageSize(first.size());

	auto heightLine = imageSize_.width;

	vector<cv::Point2d> test_subPix;

	auto sigmaY = 2;
	auto currentTest = 1;

	auto kernelMin = 3;
	auto kernelMax = 31;

	addKernelTests(tests, alphaBase, sigmaXBase, kernelMin, kernelMax);

	// auto kernel from sigma
	for (auto i = currentTest; i <= 10; ++i)
		tests.push_back(TestConfig(alphaBase, sigmaXBase * i, i, cv::Size(0, 0)));

	// start the process of gathering information for set frame count
	cv::Size blurSize(3, 3);

	const auto alpha = 0.5;
	const auto beta = 1.0 - alpha;

	auto line_fraction = 0;
	auto line_thickness = 1;

	/* erosion and dilation trackbar settings */
	auto erosion_type = 2;
	auto erosion_size = 3;

	auto dilation_type = 0;
	auto dilation_size = 1;

	auto const max_ed_elem = 2;
	auto const max_ed_kernel_size = 21;
	/* end */

	// test for video recording
	if (saveVideo_) {
		is.SetInformation(Information::Full);
		is.SetSaveType(SaveType::Video);
		is.SetCodec(VideoCodec::Mjpeg);
		is.SetFPS(25.0f);
		is.SetSize(frame.cols, frame.rows);
		is.SetColour(VideoColour::Colour);
		is.SetFileName("_testvideo");
		is.OpenVideo();
	}

	// configure output stuff
	for (auto i = frameCount_; i--;) {
		pix_Planarmap[i].reserve(imageSize_.width);
		sparse[i].reserve(imageSize_.width);
	}

	auto testSize = tests.size();

	for (auto i = testSize; i--;) {

		// start the process of gathering information for set frame count
		//	for (auto& t : tests) {
		Util::log("Running test " + to_string(currentTest) + " of " + to_string(tests.size()));
		cout << "Config : " << tests[i] << endl;

		uint64 time_begin = cv::getTickCount();

		vector<double> baseLine(frameCount_);

		//ProgressBar progressFrames(100, "Computing frames..");
		//progressFrames.SetFrequencyUpdate(10);
		//progressFrames.SetStyle(">", "-");
		//progressFrames.Progressed(0);

		for (auto j = 0; j < frameCount_; ++j) {

			outputs[j] = cv::Mat::zeros(imageSize_, CV_8UC1);
			pix_Planarmap.at(j).clear();

			frame = frames[j].clone();
			//cap >> frame;

			// do basic in-place binary threshold
			threshold(frame, frame, binaryThreshold_, 255, CV_THRESH_BINARY);

			equalizeHist(frame, frame);

			// blur in-place
			GaussianBlur(frame, frame, tests[i].kernel(), tests[i].sigma(), 2, cv::BORDER_DEFAULT);

			auto generateOk = getSparseY(frame, sparse[j]);
			if (!generateOk) {
				Util::loge("Failed to generate sparse vector for frame #" + to_string(j + 1) + " of " + to_string(frameCount_));
				continue;
			}

			// extract information from the image, and make new output based on pixel intensity mean in Y-axis for each X point
			generateOk = miniCalc.generatePlanarPixels(frame, outputs[j], pix_Planarmap.at(j), test_subPix);

			if (!generateOk) {
				Util::loge("Failed to map pixels to 2D plane for frame #" + to_string(j + 1) + " of " + to_string(frameCount_));
				continue;
			}

			// temporary disabled gab filling !!!

			//findNonZero(outputs[j], pix_Planarmap[arrayLimit - (1 + j)]);
			////gabs.push_back(miniCalc.fillElementGabs(pix_Planarmap[arrayLimit - (1 + i)], outputs[i], baseLine_));
			//gabs.push_back(miniCalc.fillElementGabs(pix_Planarmap[arrayLimit - (1 + j)], outputs[j]));
			//if (gabs.back().hasValue()) {

			//}

			auto highestPixel = outputs[j].rows - pix.getHighestYpixel(outputs[j], heightLine, miniCalc);
			auto bl = computerBaseLine(outputs[j], highestPixel);
			if (cvIsNaN(bl)) {
				Util::loge("Error while computing baseline for frame " + to_string(j));
				continue;
			}

			baseLine.push_back(bl);

			//progressFrames.Progressed(i);

		}

		// generate combined sparse vector for all computed frames
		//auto added = 0;
		//for (auto k = 0; k < frameCount_; ++k) {
		//	if (sparse[k].empty())
		//		continue;
		//	for (auto& p : sparse[k]) {
		//		
		//	}
		//}

		auto base = miniCalc.mean(baseLine);

		frame = cv::Mat::zeros(imageSize_, CV_8UC1);
		cv::Mat lines = cv::Mat::zeros(imageSize_, CV_8UC1);

		// merge the images to target
		for (auto j = 0; j < frameCount_; ++j) {
			addWeighted(outputs[j], alpha, lines, beta, 0.0, lines);
			//add(outputs[j], lines, lines);
			outputs[j].release();
			if (saveVideo_) is.SaveVideoFrame(lines);

		}

		//for (auto o = 0; o < frameCount_; ++o) {
		//	vi diffFirst, diffSecond;
		//	miniCalc.diffirentiateSparse(sparse[o], diffFirst);
		//	miniCalc.diffirentiateSparse(diffFirst, diffSecond);
		//	miniCalc.splitSparse(diffSecond, leftSideLine_, rightSideLine_, lines.cols >> 1);
		//	//Util::log("sparse: " + to_string(sparse[o].size()));
		//	//Util::log("Left: " + to_string(leftSideLine_.size()) + " Right: " + to_string(rightSideLine_.size()));
		//}


		cv::Mat output = cv::Mat::zeros(imageSize_, lines.type());

		/* test stuff for filtering out crap pixels */
		addWeighted(lines, 1.5, lines, -0.5, 0, output);

		//skeleton(&output);
		auto erosion_image = this->erosion(output, erosion_type, erosion_size);
		bilateralFilter(erosion_image, output, 1, 20, 10);
		//bilateralFilter(erosion_image, output, 1, 80, 20);
		erosion_image.release();

		/* end test stuff */

		resize(output, frame, output.size() * 2, 0, 0, cv::INTER_LANCZOS4);

		GaussianBlur(frame, output, tests[i].kernel(), tests[i].sigma(), 10, cv::BORDER_CONSTANT);

		// test for highest pixel for eroded image
		auto highestPixel = output.rows - pix.getHighestYpixel(output, heightLine, miniCalc) - base;
		//cout << "Highest Y in eroded line : " << highestPixel << " [mm: " << to_string(miniCalc.calculatePixelToMm(highestPixel)) << "]" << endl;

		// gather all elements from final matrix
		computeAllElements(output);
		computerGaugeLine(output);

		frameTime_ = cv::getTickCount() - time_begin;
		is.UpdateTimeStamp();

		//if (showWindows_) {
		//	imshow(erodeWindowName, output);
		//}

		auto timeString = to_string(getFrameTime() / getTickFrequency());
		ostringstream testInfo;
		testInfo << tests[i];
		Util::log("Saving image..");
		is.SaveImage(&output, "_test" + to_string(currentTest));
		Util::log("Saving test data..");
		savePlanarImageData("_test" + to_string(currentTest) + "_data", allPixels_, output, highestPixel, timeString, testInfo.str());

		drawVerticalLine(&output, heightLine);
		drawHorizontalLine(&output, Util::round(base));
		is.SaveImage(&output, "_test_full" + to_string(currentTest));

		Util::log("Test " + to_string(currentTest) + " completed, took " + timeString + " seconds");

		currentTest++;
	}

	Util::log("Test session completed.. a total of " + to_string(currentTest - 1) + " tests..");

	return true;

}

/**
 * Performs multi testing with different settings based on user command line input.\n
 * Performance is not considered, so don't expect magic :)\n
 * \brief Performs multi testing
 * \return true if everything went ok, false if a recoverable failure occoured.
 */
bool ThicknessGauge::testAggressive() {
	if (!cap.isOpened()) // check if we succeeded
		throw CaptureFailException("Error while attempting to open capture device.");

	Util::log("Initiating test mode.. please wait...");

	ImageSave is("test_x", SaveType::Image_Png, Information::Basic);

	// kernel size vector
	const vector<cv::Size> kernels_ = {cv::Size(0, 0), cv::Size(3, 3), cv::Size(5, 5), cv::Size(7, 7), cv::Size(9, 9)};

	// weigthed adding boundries for alpha
	// beta values are always 1.0 = alpha
	float alphaBase = 0.1f;

	// blur sigma boundries
	auto sigmaXBase = 5;

	// each test is put here, to better control the flow
	vector<TestConfig> tests;

	const auto arrayLimit = 1024; // shit c++11 ->

	cv::Mat frame;
	vector<v2<double>> gabs(frameCount_);

	array<cv::Mat, arrayLimit> frames;
	array<cv::Mat, arrayLimit> outputs;
	array<vi, arrayLimit> pix_Planarmap;

	array<vi, arrayLimit> sparse;

	vi nonZero;

	cv::Mat first;
	// capture first frame, only to get sizes etc.
	cap >> first;


	auto out("Capturing " + to_string(frameCount_) + " frames..");

	ProgressBar progress(frameCount_, out.c_str());
	progress.SetFrequencyUpdate(10);
	progress.SetStyle(">", "-");

	for (auto i = frameCount_; i--;) {
		progress.Progressed(i);
		cap >> frames[i];
	}
	progress.Progressed(frameCount_);

	Util::log("");

	cap.release();

	setImageSize(first.size());

	auto heightLine = imageSize_.width;

	vector<cv::Point2d> test_subPix;

	auto sigmaY = 2;
	auto currentTest = 1;

	auto kernelMin = 3;
	auto kernelMax = 31;

	addKernelTests(tests, alphaBase, sigmaXBase, kernelMin, kernelMax);

	// auto kernel from sigma
	for (auto i = currentTest; i <= 10; ++i)
		tests.push_back(TestConfig(alphaBase, sigmaXBase * i, i, cv::Size(0, 0)));

	// start the process of gathering information for set frame count
	cv::Size blurSize(3, 3);

	const auto alpha = 0.5;
	const auto beta = 1.0 - alpha;

	auto line_fraction = 0;
	auto line_thickness = 1;

	/* erosion and dilation trackbar settings */
	auto erosion_type = 2;
	auto erosion_size = 3;

	auto dilation_type = 0;
	auto dilation_size = 1;

	auto const max_ed_elem = 2;
	auto const max_ed_kernel_size = 21;
	/* end */

	// test for video recording
	if (saveVideo_) {
		is.SetInformation(Information::Full);
		is.SetSaveType(SaveType::Video);
		is.SetCodec(VideoCodec::Mjpeg);
		is.SetFPS(25.0f);
		is.SetSize(frame.cols, frame.rows);
		is.SetColour(VideoColour::Colour);
		is.SetFileName("_testvideo");
		is.OpenVideo();
	}

	// configure output stuff
	for (auto i = frameCount_; i--;) {
		pix_Planarmap[i].reserve(imageSize_.width);
		sparse[i].reserve(imageSize_.width);
	}

	auto testSize = tests.size();

	for (auto i = 0; i < testSize; ++i) {

		// start the process of gathering information for set frame count
		//	for (auto& t : tests) {
		Util::log("Running test " + to_string(currentTest) + " of " + to_string(tests.size()));
		cout << "Config : " << tests[i] << endl;

		uint64 time_begin = cv::getTickCount();

		vector<double> baseLine(frameCount_);

		//ProgressBar progressFrames(100, "Computing frames..");
		//progressFrames.SetFrequencyUpdate(10);
		//progressFrames.SetStyle(">", "-");
		//progressFrames.Progressed(0);

		for (auto j = 0; j < frameCount_; ++j) {

			outputs[j] = cv::Mat::zeros(imageSize_, CV_8UC1);
			pix_Planarmap.at(j).clear();

			frame = frames[j].clone();
			//cap >> frame;

			// do basic in-place binary threshold
			threshold(frame, frame, binaryThreshold_, 255, CV_THRESH_BINARY);

			equalizeHist(frame, frame);

			// blur in-place
			GaussianBlur(frame, frame, tests[i].kernel(), tests[i].sigma(), 2, cv::BORDER_DEFAULT);

			// extract information from the image, and make new output based on pixel intensity mean in Y-axis for each X point
			auto generateOk = miniCalc.generatePlanarPixels(frame, outputs[j], pix_Planarmap.at(j), test_subPix);

			if (!generateOk) {
				Util::loge("Failed to map pixels to 2D plane for frame #" + to_string(j + 1) + " of " + to_string(frameCount_));
				continue;
			}

			// temporary disabled gab filling !!!

			//findNonZero(outputs[j], pix_Planarmap[arrayLimit - (1 + j)]);
			////gabs.push_back(miniCalc.fillElementGabs(pix_Planarmap[arrayLimit - (1 + i)], outputs[i], baseLine_));
			//gabs.push_back(miniCalc.fillElementGabs(pix_Planarmap[arrayLimit - (1 + j)], outputs[j]));
			//if (gabs.back().hasValue()) {

			//}

			auto highestPixel = outputs[j].rows - pix.getHighestYpixel(outputs[j], heightLine, miniCalc);
			auto bl = computerBaseLine(outputs[j], highestPixel);
			if (cvIsNaN(bl)) {
				Util::loge("Error while computing baseline for frame " + to_string(j));
				continue;
			}

			baseLine.push_back(bl);

			//progressFrames.Progressed(i);

		}

		auto base = miniCalc.mean(baseLine);

		frame = cv::Mat::zeros(imageSize_, CV_8UC1);
		cv::Mat lines = cv::Mat::zeros(imageSize_, CV_8UC1);

		// merge the images to target
		for (auto k = frameCount_; k--;) {
			addWeighted(outputs[k], alpha, lines, beta, 0.0, lines);
			//add(outputs[k], lines, lines);
			outputs[k].release();
			if (saveVideo_) is.SaveVideoFrame(lines);
		}

		cv::Mat output = cv::Mat::zeros(imageSize_, lines.type());

		/* test stuff for filtering out crap pixels */
		addWeighted(lines, 1.5, lines, -0.5, 0, output);

		//skeleton(&output);
		auto erosion_image = this->erosion(output, erosion_type, erosion_size);
		bilateralFilter(erosion_image, output, 1, 20, 10);
		//bilateralFilter(erosion_image, output, 1, 80, 20);
		erosion_image.release();

		/* end test stuff */

		resize(output, frame, output.size() * 2, 0, 0, cv::INTER_LANCZOS4);

		GaussianBlur(frame, output, tests[i].kernel(), tests[i].sigma(), 10, cv::BORDER_CONSTANT);

		frame.release();

		// test for highest pixel for eroded image
		auto highestPixel = output.rows - pix.getHighestYpixel(output, heightLine, miniCalc) - base;
		//cout << "Highest Y in eroded line : " << highestPixel << " [mm: " << to_string(miniCalc.calculatePixelToMm(highestPixel)) << "]" << endl;

		// gather all elements from final matrix
		computeAllElements(output);
		computerGaugeLine(output);

		frameTime_ = cv::getTickCount() - time_begin;
		is.UpdateTimeStamp();

		//if (showWindows_) {
		//	imshow(erodeWindowName, output);
		//}

		auto timeString = to_string(getFrameTime() / getTickFrequency());
		ostringstream testInfo;
		testInfo << tests[i];
		Util::log("Saving image..");
		is.SaveImage(&output, "_test" + to_string(currentTest));
		Util::log("Saving test data..");
		savePlanarImageData("_test" + to_string(currentTest) + "_data", allPixels_, output, static_cast<int>(highestPixel), timeString, testInfo.str());

		drawVerticalLine(&output, heightLine);
		drawHorizontalLine(&output, Util::round(base));
		is.SaveImage(&output, "_test_full" + to_string(currentTest));

		Util::log("Test " + to_string(currentTest) + " completed, took " + timeString + " seconds");

		currentTest++;
	}

	Util::log("Test session completed.. a total of " + to_string(currentTest - 1) + " tests..");

	return true;
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

void ThicknessGauge::drawText(cv::Mat* image, const string text, TextDrawPosition position) const {
	cv::Point pos;
	switch (position) {
	case TextDrawPosition::UpperLeft:
		pos.x = image->cols / 3;
		pos.y = image->rows >> 2;
		break;
	case TextDrawPosition::UpperRight:
		pos.x = image->cols - image->cols / 3;
		pos.y = image->rows >> 2;
		break;
	case TextDrawPosition::LowerLeft:
		pos.x = image->cols - image->cols / 3;
		pos.y = image->rows - (image->rows >> 2);
		break;
	case TextDrawPosition::LowerRight:
		pos.x = image->cols - image->cols / 3;
		pos.y = image->rows - (image->rows >> 2);
		break;
	default:
		// oh noes..
		break;
	}
	putText(*image, text, pos, 1, 1.0, baseColour_, 2);
}

void ThicknessGauge::drawHorizontalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
	//cout << "line drawn at : " << pos << endl;
	line(*image, cv::Point(0, image->rows - pos), cv::Point(image->cols, image->rows - pos), colour, 1, cv::LINE_AA);
}

void ThicknessGauge::drawVerticalLine(cv::Mat* image, uint pos, cv::Scalar colour) {
	line(*image, cv::Point(pos, 0), cv::Point(pos, image->rows), colour);
}

void ThicknessGauge::drawCenterAxisLines(cv::Mat* image, cv::Scalar& colour) {
	line(*image, cv::Point(0, image->rows >> 1), cv::Point(image->cols, image->rows >> 1), colour);
	line(*image, cv::Point(image->cols >> 1, 0), cv::Point(image->cols >> 1, image->rows), colour);
}

void ThicknessGauge::drawHorizontalLine(cv::Mat* image, uint pos) const {
	drawHorizontalLine(image, pos, baseColour_);
}

void ThicknessGauge::drawVerticalLine(cv::Mat* image, uint pos) const {
	drawVerticalLine(image, pos, baseColour_);
}

[[deprecated("Not viable in the current setup.")]]
void ThicknessGauge::GenerateInputQuad(cv::Mat* image, cv::Point2f* quad) {
	// The 4 points that select quadilateral on the input , from top-left in clockwise order
	// These four pts are the sides of the rect box used as input
	quad[0] = cv::Point2f(0.0f, 0.0f);
	quad[1] = cv::Point2f(static_cast<float>(image->cols), 0.0f);
	quad[2] = cv::Point2f(static_cast<float>(image->cols), static_cast<float>(image->rows));
	quad[3] = cv::Point2f(0.0f, static_cast<float>(image->rows));
}

[[deprecated("Not viable in the current setup.")]]
void ThicknessGauge::GenerateOutputQuad(cv::Mat* image, cv::Point2f* quad) {
	// The 4 points where the mapping is to be done , from top-left in clockwise order
	quad[0] = cv::Point2f(-image->cols / 2.0f, 0.0f);
	quad[1] = cv::Point2f(static_cast<float>(image->cols) + image->cols / 2.0f, 0.0f);
	quad[2] = cv::Point2f(static_cast<float>(image->cols), static_cast<float>(image->rows));
	quad[3] = cv::Point2f(0.0f, static_cast<float>(image->rows));
}

[[deprecated("Not viable in the current setup.")]]
void ThicknessGauge::FitQuad(cv::Mat* image, cv::Point2f* inputQuad, cv::Point2f* outputQuad) const {
	// calculate transformation
	cv::Matx33f M = getPerspectiveTransform(inputQuad, outputQuad);

	// calculate warped position of all corners
	auto a = M.inv() * cv::Point3f(0.0f, 0.0f, 1.0f);
	auto b = M.inv() * cv::Point3f(0.0f, static_cast<float>(image->rows), 1.0f);
	auto c = M.inv() * cv::Point3f(static_cast<float>(image->cols), static_cast<float>(image->rows), 1.0f);
	auto d = M.inv() * cv::Point3f(static_cast<float>(image->cols), 0.0f, 1.0f);

	a *= (1.0f / a.z);
	b *= (1.0f / b.z);
	c *= (1.0f / c.z);
	d *= (1.0f / d.z);

	// to make sure all corners are in the image, every position must be > (0, 0)
	auto x = ceil(abs(min(min(a.x, b.x), min(c.x, d.x))));
	auto y = ceil(abs(min(min(a.y, b.y), min(c.y, d.y))));

	// and also < (width, height)
	auto width = ceil(abs(max(max(a.x, b.x), max(c.x, d.x)))) + x;
	auto height = ceil(abs(max(max(a.y, b.y), max(c.y, d.y)))) + y;

	// adjust target points accordingly
	for (auto i = 0; i < 4; i++)
		inputQuad[i] += cv::Point2f(x, y);

	// recalculate transformation
	M = getPerspectiveTransform(inputQuad, outputQuad);

	// get result
	cv::Mat result;
	warpPerspective(*image, result, M, cv::Size(static_cast<int>(width), static_cast<int>(height)), cv::WARP_INVERSE_MAP);

	imshow("quadfit", result);

	cv::waitKey(3);
}

void ThicknessGauge::WarpImage(cv::Mat* input, cv::Mat* output) {
}

void ThicknessGauge::WarpMeSomeCookies(cv::Mat* image, cv::Mat* output) {
	vector<cv::Point2f> points2D;
	points2D.push_back(cv::Point2f(0, 0));
	points2D.push_back(cv::Point2f(50, 0));
	points2D.push_back(cv::Point2f(50, 50));
	points2D.push_back(cv::Point2f(0, 50));

	//cv::Mat perspectiveMat = cv::getPerspectiveTransform(points2D, *image);
	//cv::warpPerspective(*_image, *_undistortedImage, M, cv::Size(_image->cols, _image->rows));
}

cv::Mat ThicknessGauge::cornerHarris_test(cv::Mat& image, int threshold) const {

	cv::Mat dst_norm, dst_norm_scaled;
	cv::Mat dst = cv::Mat::zeros(image.size(), CV_32FC1);

	/// Detector parameters
	auto blockSize = 2;
	auto apertureSize = 3;
	auto k = 0.04;

	/// Detecting corners
	cornerHarris(image, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

	/// Normalizing
	normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
	convertScaleAbs(dst_norm, dst_norm_scaled);

	/// Drawing a circle around corners
	for (auto j = 0; j < dst_norm.rows; j++) {
		for (auto i = 0; i < dst_norm.cols; i++) {
			if (static_cast<int>(dst_norm.at<float>(j, i)) > threshold)
				circle(dst_norm_scaled, cv::Point(i, j), 5, cv::Scalar(0), 2, 8, 0);
		}
	}
	return dst_norm_scaled;
}

cv::Mat ThicknessGauge::erosion(cv::Mat& input, int element, int size) const {
	cv::MorphShapes erosion_type;
	if (element == 0)
		erosion_type = cv::MORPH_RECT;
	else if (element == 1)
		erosion_type = cv::MORPH_CROSS;
	else if (element == 2)
		erosion_type = cv::MORPH_ELLIPSE;
	else
		erosion_type = cv::MORPH_RECT;

	auto input_element = getStructuringElement(erosion_type, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));

	cv::Mat erosion_dst = cv::Mat::zeros(input.size(), input.type());
	erode(input, erosion_dst, element, cv::Point(-1, -1), 1, cv::BORDER_DEFAULT);
	return erosion_dst;
}

cv::Mat ThicknessGauge::dilation(cv::Mat& input, int dilation, int size) const {
	cv::MorphShapes dilation_type;
	if (dilation == 0)
		dilation_type = cv::MORPH_RECT;
	else if (dilation == 1)
		dilation_type = cv::MORPH_CROSS;
	else if (dilation == 2)
		dilation_type = cv::MORPH_ELLIPSE;
	else
		dilation_type = cv::MORPH_RECT;

	auto element = getStructuringElement(dilation_type, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
	cv::Mat dilation_dst = cv::Mat::zeros(input.size(), input.type());

	dilate(input, dilation_dst, element);
	return dilation_dst;
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
}
