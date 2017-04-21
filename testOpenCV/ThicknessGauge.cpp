#include <algorithm>
#include "ThicknessGauge.h"
#include "MiniCalc.h"
#include "Util.h"
#include "ImageSave.h"
#include "CaptureFailException.h"

void ThicknessGauge::gatherPixels(Mat& image) {
	findNonZero(image, pixels_);

	for (auto& pixel_point : pixels_) {
		if (pixel_point.x < 50 && IsPixel(image, pixel_point)) {
			rightSideLine_.push_back(pixel_point);
		}
		else if (pixel_point.x <= image.cols - 50 && IsPixel(image, pixel_point)) {
			leftSideLine_.push_back(pixel_point);
		}
	}
}

void ThicknessGauge::laplace(Mat& image) const {
	Mat tmp;
	Laplacian(image, tmp, ddepth_, kernelSize_); // , scale, delta, BORDER_DEFAULT);
	convertScaleAbs(tmp, image);
}

void ThicknessGauge::sobel(Mat& image) const {
	Sobel(image, image, -1, 1, 1, kernelSize_, scale_, delta_, BORDER_DEFAULT);
}

void ThicknessGauge::imageSkeleton(Mat& image) {
	//cv::threshold(image, image, 200, 255, cv::THRESH_BINARY);
	Mat skel(image.size(), CV_8UC1, Scalar(0));
	Mat temp;
	Mat eroded;

	auto element = getStructuringElement(MORPH_CROSS, Size(6, 6));

	for (;;) {
		erode(image, eroded, element);
		dilate(eroded, temp, element); // temp = open(img)
		subtract(image, temp, temp);
		bitwise_or(skel, temp, skel);
		eroded.copyTo(image);

		if (countNonZero(image) == 0)
			break;
	}

}

void ThicknessGauge::drawPlarnarPixels(Mat& targetImage, vector<Point>& planarMap) const {
	polylines(targetImage, planarMap, false, Scalar(255, 255, 255), 2);
}

int ThicknessGauge::getHighestYpixel(Mat& image) {
	auto highest = image.rows;
	vector<Point> pix;
	findNonZero(image, pix);
	for (auto& p : pix) {
		if (p.y < highest)
			highest = p.y;
	}
	return highest;
}

bool ThicknessGauge::generatePlanarImage() {
	if (!cap.isOpened()) // check if we succeeded
		throw CaptureFailException("Error while attempting to open capture device.");

	Point textPoint(100, 100);
	Size blurSize(5, 5);
	const Scalar baseColour(255, 255, 255);
	const auto alpha = 0.2;
	const auto beta = 1.0 - alpha;

	auto thres = 254;
	auto line_fraction = 0;
	auto line_thickness = 1;

	/* erosion and dilation trackbar settings */
	auto erosion_type = 0;
	auto erosion_size = 1;

	auto dilation_type = 0;
	auto dilation_size = 1;

	auto const max_ed_elem = 2;
	auto const max_ed_kernel_size = 21;
	/* end */

	ImageSave is("pic_x", SaveType::Image_Png, Information::Basic);

	const string inputWindowName = "GC2450 feed";
	const string outputWindowName = "GC2450 manipulated";
	const string line1WindowName = "GC2450 singular frame mean intensity";
	const string line2WindowName = "GC2450 raw adding of means over time";
	const string line3WindowName = "GC2450 weighted means over time";
	const string cornerWindowName = "GC2450 Corner View";
	const string erodeWindowName = "Erosion";
	const string dilationWindowName = "Dilation";
	if (showWindows_) {
		namedWindow(inputWindowName, WINDOW_FREERATIO);

		namedWindow(outputWindowName, WINDOW_FREERATIO);
		createTrackbar("Threshold", outputWindowName, &thres, 254);

		namedWindow(line1WindowName, WINDOW_FREERATIO);
		createTrackbar("Frac", line1WindowName, &line_fraction, 4);
		createTrackbar("Thick", line1WindowName, &line_thickness, 5);

		namedWindow(line2WindowName, WINDOW_FREERATIO);

		namedWindow(line3WindowName, WINDOW_FREERATIO);

		namedWindow(cornerWindowName, WINDOW_FREERATIO);

		namedWindow(erodeWindowName, CV_WINDOW_AUTOSIZE);
		createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", erodeWindowName, &erosion_type, max_ed_elem);
		createTrackbar("Kernel size:\n 2n +1", erodeWindowName, &erosion_size, max_ed_kernel_size);

		namedWindow(dilationWindowName, CV_WINDOW_AUTOSIZE);
		createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", dilationWindowName, &dilation_type, max_ed_elem);
		createTrackbar("Kernel size:\n 2n +1", dilationWindowName, &dilation_size, max_ed_kernel_size);
	}

	Mat frame;
	Mat outputs[1024];
	vi pix_planarMap[1024]; // shit c++11 ->
	vi nonZero;

	// capture first frame
	cap >> frame;

	setImageSize(frame.size());

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

	vector<Point2d> test_subPix;

	// configure output stuff
	for (auto i = 0; i < frameCount_; ++i) {
		pix_planarMap[i].reserve(imageSize_.width);
	}

	// start the process of gathering information for set frame count

	while (true) {

		uint64 time_begin = getTickCount();

		for (auto i = 0; i < frameCount_; ++i) {


			outputs[i] = Mat::zeros(imageSize_, CV_8UC1);
			pix_planarMap[i].clear();

			cap >> frame;

			// show default input image (always shown live!)
			if (showWindows_) imshow(inputWindowName, frame);

			// do basic in-place binary threshold
			threshold(frame, frame, thres, 255, CV_THRESH_BINARY);

			// blur in-place
			GaussianBlur(frame, frame, blurSize, 0, 0, BORDER_DEFAULT);

			// perform some stuff
			laplace(frame);
			//c.Sobel(frame);

			if (showWindows_) imshow(outputWindowName, frame);

			// extract information from the image, and make new output based on pixel intensity mean in Y-axis for each X point
			auto generateOk = miniCalc.generatePlanarPixels(frame, outputs[i], pix_planarMap[i], test_subPix);

			if (!generateOk) {
				cout << "Failed to map pixels to 2D plane for frame #" << to_string(i + 1) << " of " << to_string(frameCount_) << endl;
				break;
			}

			if (showWindows_) imshow(line1WindowName, outputs[i]);

			//cout << pix_planarMap[i] << endl;

		}

		frame = Mat::zeros(imageSize_, CV_8UC1);
		Mat lines = Mat::zeros(imageSize_, CV_8UC1);

		// merge the images to target
		for (auto i = 0; i < frameCount_; ++i) {
			addWeighted(outputs[i], alpha, lines, beta, 0.0, lines);
			add(outputs[i], frame, frame);
			is.SaveVideoFrame(lines);
		}

		Mat output = Mat::zeros(imageSize_, CV_8UC1);

		bilateralFilter(frame, output, 1, 80, 20);

		if (showWindows_) imshow(line2WindowName, output);
		if (showWindows_) imshow(line3WindowName, lines);
		is.UpdateTimeStamp();
		is.SaveImage(&output);

		auto corner_image = cornerHarris_test(lines, 200);
		if (showWindows_) imshow(cornerWindowName, corner_image);

		equalizeHist(lines, lines);

		auto erosion_image = this->erosion(lines, erosion_type, erosion_size);
		resize(erosion_image, frame, erosion_image.size() * 2, 0, 0, INTER_LANCZOS4);
		if (showWindows_) imshow(erodeWindowName, frame);

		// test for highest pixel for eroded image
		cout << "Highest Y in eroded line : " << frame.rows - getHighestYpixel(frame) << endl;

		//Mat dilation = c.dilation(lines, dilation_type, dilation_size);
		//imshow(dilationWindowName, dilation);

		vi eroded_pixels;
		findNonZero(frame, eroded_pixels);

		// generate the solar ray vectors..
		generateVectors(eroded_pixels);

		frameTime_ = getTickCount() - time_begin;

		cout << "Saving image...\n";
		is.SaveImage(&frame, "_testoutput" + to_string(frameCount_));

		savePlanarImageData("_testoutput", eroded_pixels, frame, frame.rows - getHighestYpixel(frame));

		if (showWindows_) if (waitKey(25) >= 0) break;

	}
	if (showWindows_) {
		destroyWindow(inputWindowName);
		destroyWindow(outputWindowName);
		destroyWindow(line1WindowName);
		destroyWindow(line2WindowName);
		destroyWindow(line3WindowName);
	}

	if (saveVideo_)
		is.CloseVideo();

	return true;
}

bool ThicknessGauge::savePlanarImageData(string filename, vector<Point>& pixels, Mat& image, int highestY) const {
	FileStorage fs(filename + to_string(frameCount_) + ".json", FileStorage::WRITE_BASE64);

	if (!fs.isOpened()) {
		cerr << "Error while opening " << filename << " for output." << endl;
		return false;
	}

	ostringstream oss;
	oss << Util::getTime();
	cout << "Saving data [frames: " << frameCount_ << "] [time:" << oss.str() << "] [pixels:" << pixels.size() << ']' << endl;

	fs << "Original filename" << filename;
	fs << "Saved time" << Util::getTime();
	fs << "Seconds for computation" << static_cast<long>(frameTime_ / tickFrequency_);
	fs << "Highest Y" << highestY;

	fs << "Eroded pixel count" << static_cast<int>(pixels.size());
	fs << "Eroded pixels" << pixels;
	fs << "Eroded image" << image;

	fs.release();

	//ImageSave is(filename, SaveType::Image_Png);
	//is.SaveImage(image);

	return true;
}

void ThicknessGauge::setRightMean(double mean) {
	rightMean_ = mean;
}

double ThicknessGauge::getRightMean() const {
	return rightMean_;
}

void ThicknessGauge::setLeftMean(double mean) {
	leftMean_ = mean;
}

double ThicknessGauge::getLeftMean() const {
	return leftMean_;
}

void ThicknessGauge::drawText(Mat* image, const string text, TextDrawPosition position) {
	Point pos;
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
	putText(*image, text, pos, 1, 1.0, CV_RGB(0, 0, 0), 2);
}

void ThicknessGauge::drawCenterAxisLines(Mat* image) {
	line(*image, cvPoint(0, image->rows >> 1), cvPoint(image->cols, image->rows >> 1), CV_RGB(255, 255, 255));
	line(*image, cvPoint(image->cols >> 1, 0), cvPoint(image->cols >> 1, image->rows), CV_RGB(255, 255, 255));
}

void ThicknessGauge::GenerateInputQuad(Mat* image, Point2f* quad) {
	// The 4 points that select quadilateral on the input , from top-left in clockwise order
	// These four pts are the sides of the rect box used as input
	quad[0] = Point2f(0.0f, 0.0f);
	quad[1] = Point2f(static_cast<float>(image->cols), 0.0f);
	quad[2] = Point2f(static_cast<float>(image->cols), static_cast<float>(image->rows));
	quad[3] = Point2f(0.0f, static_cast<float>(image->rows));
}

void ThicknessGauge::GenerateOutputQuad(Mat* image, Point2f* quad) {
	// The 4 points where the mapping is to be done , from top-left in clockwise order
	quad[0] = Point2f(-image->cols / 2.0f, 0.0f);
	quad[1] = Point2f(static_cast<float>(image->cols) + image->cols / 2.0f, 0.0f);
	quad[2] = Point2f(static_cast<float>(image->cols), static_cast<float>(image->rows));
	quad[3] = Point2f(0.0f, static_cast<float>(image->rows));
}

void ThicknessGauge::FitQuad(Mat* image, Point2f* inputQuad, Point2f* outputQuad) const {
	// calculate transformation
	Matx33f M = getPerspectiveTransform(inputQuad, outputQuad);

	// calculate warped position of all corners
	auto a = M.inv() * Point3f(0.0f, 0.0f, 1.0f);
	auto b = M.inv() * Point3f(0.0f, static_cast<float>(image->rows), 1.0f);
	auto c = M.inv() * Point3f(static_cast<float>(image->cols), static_cast<float>(image->rows), 1.0f);
	auto d = M.inv() * Point3f(static_cast<float>(image->cols), 0.0f, 1.0f);

	a *= (1.0f / a.z);
	b *= (1.0f / b.z);
	c *= (1.0f / c.z);
	d *= (1.0f / d.z);

	// to make sure all corners are in the image, every position must be > (0, 0)
	auto x = ceil(abs(std::min(std::min(a.x, b.x), std::min(c.x, d.x))));
	auto y = ceil(abs(std::min(std::min(a.y, b.y), std::min(c.y, d.y))));

	// and also < (width, height)
	auto width = ceil(abs(std::max(std::max(a.x, b.x), std::max(c.x, d.x)))) + x;
	auto height = ceil(abs(std::max(std::max(a.y, b.y), std::max(c.y, d.y)))) + y;

	// adjust target points accordingly
	for (auto i = 0; i < 4; i++) {
		inputQuad[i] += Point2f(x, y);
	}

	// recalculate transformation
	M = getPerspectiveTransform(inputQuad, outputQuad);

	// get result
	Mat result;
	warpPerspective(*image, result, M, Size(static_cast<int>(width), static_cast<int>(height)), WARP_INVERSE_MAP);

	imshow("quadfit", result);

	waitKey(3);
}

void ThicknessGauge::WarpImage(Mat* input, Mat* output) {
}

void ThicknessGauge::WarpMeSomeCookies(Mat* image, Mat* output) {
	vector<Point2f> points2D;
	points2D.push_back(Point2f(0, 0));
	points2D.push_back(Point2f(50, 0));
	points2D.push_back(Point2f(50, 50));
	points2D.push_back(Point2f(0, 50));

	//cv::Mat perspectiveMat = cv::getPerspectiveTransform(points2D, *image);
	//cv::warpPerspective(*_image, *_undistortedImage, M, cv::Size(_image->cols, _image->rows));
}

Mat ThicknessGauge::cornerHarris_test(Mat& image, int threshold) const {

	Mat dst_norm, dst_norm_scaled;
	Mat dst = Mat::zeros(image.size(), CV_32FC1);

	/// Detector parameters
	auto blockSize = 2;
	auto apertureSize = 3;
	auto k = 0.04;

	/// Detecting corners
	cornerHarris(image, dst, blockSize, apertureSize, k, BORDER_DEFAULT);

	/// Normalizing
	normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(dst_norm, dst_norm_scaled);

	/// Drawing a circle around corners
	for (auto j = 0; j < dst_norm.rows; j++) {
		for (auto i = 0; i < dst_norm.cols; i++) {
			if (static_cast<int>(dst_norm.at<float>(j, i)) > threshold)
				circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
		}
	}
	return dst_norm_scaled;
}

Mat ThicknessGauge::erosion(Mat& input, int element, int size) const {
	auto erosion_type = 0;
	if (element == 0) {
		erosion_type = MORPH_RECT;
	}
	else if (element == 1) {
		erosion_type = MORPH_CROSS;
	}
	else if (element == 2) {
		erosion_type = MORPH_ELLIPSE;
	}

	auto input_element = getStructuringElement(erosion_type, Size(2 * size + 1, 2 * size + 1), Point(size, size));

	Mat erosion_dst = Mat::zeros(input.size(), input.type());
	erode(input, erosion_dst, element);
	return erosion_dst;
}

Mat ThicknessGauge::dilation(Mat& input, int dilation, int size) const {
	auto dilation_type = 0;
	if (dilation == 0) {
		dilation_type = MORPH_RECT;
	}
	else if (dilation == 1) {
		dilation_type = MORPH_CROSS;
	}
	else if (dilation == 2) {
		dilation_type = MORPH_ELLIPSE;
	}

	auto element = getStructuringElement(dilation_type, Size(2 * size + 1, 2 * size + 1), Point(size, size));
	Mat dilation_dst = Mat::zeros(input.size(), input.type());

	dilate(input, dilation_dst, element);
	return dilation_dst;
}

const vi& ThicknessGauge::getPixels() const {
	return pixels_;
}

const vd& ThicknessGauge::getAllPixels() const {
	return allPixels_;
}

const vd& ThicknessGauge::getMeasureLine() const {
	return measureLine_;
}

const vd& ThicknessGauge::getRightSideLine() const {
	return rightSideLine_;
}

const vd& ThicknessGauge::getLeftSideLine() const {
	return leftSideLine_;
}

int ThicknessGauge::getFrameCount() const {
	return frameCount_;
}

void ThicknessGauge::setFrameCount(int frameCount) {
	frameCount_ = frameCount;
}

uint64 ThicknessGauge::getFrameTime() const {
	return frameTime_;
}

void ThicknessGauge::setFrameTime(uint64 uint64) {
	frameTime_ = uint64;
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

Mat& ThicknessGauge::GetPlanarImage() {
	return planarImage_;
}

void ThicknessGauge::setPlanarImage(const Mat& mat) {
	planarImage_ = mat;
}
