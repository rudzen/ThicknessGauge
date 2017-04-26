#include <algorithm>
#include "ThicknessGauge.h"
#include "MiniCalc.h"
#include "Util.h"
#include "ImageSave.h"
#include "CaptureFailException.h"
#include "Specs.h"

int ThicknessGauge::getBaseLine() const {
	return baseLine_;
}

void ThicknessGauge::setBaseLine(int baseLine) {
	baseLine_ = baseLine;
}

void ThicknessGauge::initVideoCapture() {
	cap.open(CV_CAP_PVAPI);
}

void ThicknessGauge::initCalibrationSettings(string fileName) {
	cs.readSettings(fileName);
}

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

void ThicknessGauge::Blur(Mat& image, Size size) {
	GaussianBlur(image, image, size, 1.5, 1.5);
}

void ThicknessGauge::MeanReduction(Mat& image) {
	MeanReduction(image);
}

void ThicknessGauge::laplace(Mat& image) const {
	Mat tmp;
	Laplacian(image, tmp, settings.ddepth, settings.kernelSize); // , scale, delta, BORDER_DEFAULT);
	convertScaleAbs(tmp, image);
}

void ThicknessGauge::sobel(Mat& image) const {
	Sobel(image, image, -1, 1, 1, settings.kernelSize, settings.scale, settings.delta, BORDER_DEFAULT);
}

void ThicknessGauge::skeleton(Mat& image) {
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

		if (countNonZero(image) < 25000)
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

int ThicknessGauge::getHighestYpixel(Mat& image, int x) const {
	auto highest = image.rows;
	vector<Point> pix;
	findNonZero(image, pix);

	sort(pix.begin(), pix.end(), miniCalc.sortX);

	for (auto& p : pix) {
		if (p.x < x)
			continue;
		if (p.x > x)
			break;
		if (p.y < highest)
			highest = p.y;
	}
	return highest;
}

int ThicknessGauge::getAllPixelSum(Mat& image) {

	auto sum = 0;
	auto uc_pixel = image.data;
	for (auto row = 0; row < image.rows; ++row) {
		uc_pixel = image.data + row * image.step;
		for (auto col = 0; col < image.cols; ++col) {
			int a = uc_pixel[0];
			int b = uc_pixel[1];
			int c = uc_pixel[2];
			sum += a + b + c;
			uc_pixel += 3;
		}
	}
	return sum;
}

double ThicknessGauge::getYPixelsAvg(Mat& image, int x) {
	auto sum = 0;
	auto count = 0;
	auto col = image.col(x);
	auto uc_pixel = col.data;

	for (auto i = 0; i < col.cols; ++i) {
		sum += uc_pixel[0];
		cout << "sum is now : " << sum << "\n";
		count++;
		uc_pixel++;
	}

	return sum / count;
}

inline void ThicknessGauge::computeAllElements(Mat& image) {
	findNonZero(image, allPixels_);
}

bool ThicknessGauge::generatePlanarImage() {
	if (!cap.isOpened()) // check if we succeeded
		throw CaptureFailException("Error while attempting to open capture device.");

	Point textPoint(100, 100);
	Size blurSize(3, 3);

	const auto alpha = 0.5;
	const auto beta = 1.0 - alpha;

	auto thres = binaryThreshold_; // default threshold.
	//auto baseLine = baseLine_; // baseline default.

	auto line_fraction = 0;
	auto line_thickness = 1;

	/* erosion and dilation trackbar settings */
	auto erosion_type = 1;
	auto erosion_size = 3;

	auto dilation_type = 0;
	auto dilation_size = 1;

	auto const max_ed_elem = 2;
	auto const max_ed_kernel_size = 21;
	/* end */

	ImageSave is("pic_x", SaveType::Image_Png, Information::Basic);

	const int arrayLimit = 512; // shit c++11 ->

	Mat frame;
	Mat outputs[arrayLimit];
	vi pix_planarMap[arrayLimit];
	vi nonZero;

	// capture first frame
	cap >> frame;

	setImageSize(frame.size());


	auto heightLine = imageSize_.width;

	const string inputWindowName = "GC2450 feed";
	const string outputWindowName = "GC2450 manipulated";
	const string line1WindowName = "GC2450 singular frame mean intensity";
	const string line2WindowName = "GC2450 raw adding of means over time";
	const string line3WindowName = "GC2450 weighted means over time";
	const string cornerWindowName = "GC2450 Corner View";
	const string erodeWindowName = "Erosion";
	const string dilationWindowName = "Dilation";
	if (showWindows_) {
		namedWindow(inputWindowName, WINDOW_AUTOSIZE);
		createTrackbar("Threshold", inputWindowName, &thres, 254);
		createTrackbar("Base Line", inputWindowName, &baseLine_, imageSize_.height);
		createTrackbar("Height Line", inputWindowName, &heightLine, (imageSize_.width * 2) - 1);

		namedWindow(outputWindowName, WINDOW_AUTOSIZE);

		//namedWindow(line1WindowName);
		////createTrackbar("Frac", line1WindowName, &line_fraction, 4);
		////createTrackbar("Thick", line1WindowName, &line_thickness, 5);

		//namedWindow(line2WindowName, WINDOW_AUTOSIZE);

		//namedWindow(line3WindowName, WINDOW_AUTOSIZE);

		//namedWindow(cornerWindowName, WINDOW_AUTOSIZE);

		namedWindow(erodeWindowName, WINDOW_AUTOSIZE);
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

			equalizeHist(frame, frame);

			// blur in-place
			GaussianBlur(frame, frame, Size(7,5), 10, 2, BORDER_DEFAULT);

			// perform some stuff
			//laplace(frame);
			// c.Sobel(frame);

			if (showWindows_) imshow(outputWindowName, frame);

			// extract information from the image, and make new output based on pixel intensity mean in Y-axis for each X point
			auto generateOk = miniCalc.generatePlanarPixels(frame, outputs[i], pix_planarMap[i], test_subPix);

			if (!generateOk) {
				cout << "Failed to map pixels to 2D plane for frame #" << to_string(i + 1) << " of " << to_string(frameCount_) << endl;
				break;
			}

			findNonZero(outputs[i], pix_planarMap[arrayLimit - (1 + i)]);
			if (miniCalc.fillElementGabs(pix_planarMap[arrayLimit - (1 + i)], outputs[i], outputs[i].rows - baseLine_)) {
				//Mat temp(pix_planarMap[arrayLimit - (1 + i)]);
				//cout << "TEMP cols " << temp.cols << endl;
				//imshow("temp", temp);
				//add(outputs[i], temp, outputs[i]);
			}

			if (showWindows_) imshow(line1WindowName, outputs[i]);

		}

		frame = Mat::zeros(imageSize_, CV_8UC1);
		Mat lines = Mat::zeros(imageSize_, CV_8UC1);

		// merge the images to target
		for (auto i = 0; i < frameCount_; ++i) {
			addWeighted(outputs[i], alpha, lines, beta, 0.0, lines);
			//add(outputs[i], frame, frame);
			if (saveVideo_) is.SaveVideoFrame(lines);
		}


		Mat output = Mat::zeros(imageSize_, lines.type());

		//if (showWindows_) imshow(line2WindowName, output);
		//if (showWindows_) imshow(line3WindowName, lines);

		//is.UpdateTimeStamp();
		//is.SaveImage(&lines);

		//auto corner_image = cornerHarris_test(lines, 200);
		//if (showWindows_) imshow(cornerWindowName, corner_image);

		/* test stuff for filtering out crap pixels */
		addWeighted(lines, 1.5, lines, -0.5, 0, lines);

		auto erosion_image = this->erosion(lines, erosion_type, erosion_size);
		bilateralFilter(erosion_image, output, 1, 80, 20);

		/* end test stuff */
		
		resize(output, frame, output.size() * 2, 0, 0, INTER_LANCZOS4);

		GaussianBlur(frame, output, blurSize, 10, 10, BORDER_CONSTANT);

		//resize(output, frame, frame.size() / 2, 0, 0, INTER_LANCZOS4);

		// test for highest pixel for eroded image
		double highestPixel = frame.rows - getHighestYpixel(output, heightLine) - baseLine_;
		cout << "Highest Y in eroded line : " << highestPixel << " [mm: " << to_string(miniCalc.calculatePixelToMm(highestPixel)) << "]" << endl;

		//Mat dilation = c.dilation(lines, dilation_type, dilation_size);
		//imshow(dilationWindowName, dilation);

		// gather all elements from final matrix
		computeAllElements(output);

		computerGaugeLine(output);

		if (showWindows_) {
			Specs s;
			s.getPixelStrengths(output, allPixels_, heightLine);
			auto lulu = s.getNonBaseLine(output, baseLine_);
			line(output, Point(0, lulu), Point(output.cols, lulu), baseColour_);
			line(output, Point(heightLine, 0), Point(heightLine, output.rows), baseColour_);

			drawHorizontalLine(&output, baseLine_, baseColour_);


			// draw the gauge line


		}

		if (showWindows_) imshow(erodeWindowName, output);

		frameTime_ = getTickCount() - time_begin;

		//cout << "Y avr for heightline : " << getYPixelsAvg(frame, heightLine) << endl;


		//cout << "Saving image...\n";
		//is.SaveImage(&frame, "_testoutput" + to_string(frameCount_));
		//savePlanarImageData("_testoutput", eroded_pixels, frame, frame.rows - getHighestYpixel(frame));

		if (showWindows_) {
			auto key = static_cast<char>(waitKey(1));
			if (key == 27)
				break; // esc
		}
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

	fs << "Eroded element count" << static_cast<int>(pixels.size());
	fs << "Eroded elements" << pixels;
	fs << "Eroded image" << image;

	fs.release();

	//ImageSave is(filename, SaveType::Image_Png);
	//is.SaveImage(image);

	return true;
}

double ThicknessGauge::sumColumn(Mat& image, int x) {

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

void ThicknessGauge::sumColumns(Mat& image, Mat& target) {

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

void ThicknessGauge::computerGaugeLine(Mat& output) {
	vi aboveLine;

	if (miniCalc.getActualPixels(allPixels_, aboveLine, baseLine_, output.rows)) {
		cout << "Retrived " << aboveLine.size() << " elements above line.\n";
		if (miniCalc.computerCompleteLine(aboveLine, gaugeLine_, lineConfig_)) {
			cout << "Computed line fitting... " << gaugeLine_ << "\n";

			gaugeLineSet_ = true;

			// sort the elements for quick access to first and last (outer points in line)
			sort(aboveLine.begin(), aboveLine.end(), miniCalc.sortX);

			avgGaugeHeight_ = gaugeLine_[3];

			if (showWindows_) {
				line(output, Point2f(aboveLine.front().x + gaugeLine_[0], gaugeLine_[3]), Point2f(aboveLine.back().x, gaugeLine_[3]), baseColour_, 1, LINE_AA);
				cout << "Average line height : " << output.rows - avgGaugeHeight_ << " elements.\n";
			}
		} else {
			gaugeLineSet_ = false;
			cout << "Failed to generate fitted line.\n";
		}
	} else {
		gaugeLineSet_ = false;
		cout << "Failed to retrive elements above line.\n";
	}

}

int ThicknessGauge::getBinaryThreshold() const {
	return binaryThreshold_;
}

void ThicknessGauge::setBinaryThreshold(int binaryThreshold) {
	binaryThreshold_ = binaryThreshold;
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

void ThicknessGauge::drawHorizontalLine(Mat* image, unsigned int pos, Scalar colour) {
	line(*image, Point(0, image->rows - pos), Point(image->cols, image->rows - pos), colour);
}

void ThicknessGauge::drawCenterAxisLines(Mat* image, Scalar& colour) {
	line(*image, cvPoint(0, image->rows >> 1), cvPoint(image->cols, image->rows >> 1), colour);
	line(*image, cvPoint(image->cols >> 1, 0), cvPoint(image->cols >> 1, image->rows), colour);
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
	for (auto i = 0; i < 4; i++)
		inputQuad[i] += Point2f(x, y);

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
	MorphShapes erosion_type;
	if (element == 0)
		erosion_type = MORPH_RECT;
	else if (element == 1)
		erosion_type = MORPH_CROSS;
	else if (element == 2)
		erosion_type = MORPH_ELLIPSE;
	else
		erosion_type = MORPH_RECT;

	auto input_element = getStructuringElement(erosion_type, Size(2 * size + 1, 2 * size + 1), Point(size, size));

	Mat erosion_dst = Mat::zeros(input.size(), input.type());
	erode(input, erosion_dst, element, Point(-1, -1), 1, BORDER_DEFAULT);
	return erosion_dst;
}

Mat ThicknessGauge::dilation(Mat& input, int dilation, int size) const {
	MorphShapes dilation_type;
	if (dilation == 0)
		dilation_type = MORPH_RECT;
	else if (dilation == 1)
		dilation_type = MORPH_CROSS;
	else if (dilation == 2)
		dilation_type = MORPH_ELLIPSE;
	else
		dilation_type = MORPH_RECT;

	auto element = getStructuringElement(dilation_type, Size(2 * size + 1, 2 * size + 1), Point(size, size));
	Mat dilation_dst = Mat::zeros(input.size(), input.type());

	dilate(input, dilation_dst, element);
	return dilation_dst;
}

int ThicknessGauge::autoBinaryThreshold(unsigned int pixelLimit) {

	if (!cap.isOpened()) // check if we succeeded
		throw CaptureFailException("Error while attempting to open capture device.");

	MiniCalc miniCalc;

	auto targetReached = false;
	auto currentThreshold = 1;

	const auto maxValue = 254;

	const Size blurSize(5, 5);
	const auto alpha = 0.2;
	const auto beta = 1.0 - alpha;

	auto thres = 250;
	auto line_fraction = 0;
	auto line_thickness = 1;

	/* erosion and dilation trackbar settings */
	auto erosion_type = 1;
	auto erosion_size = 3;

	auto dilation_type = 0;
	auto dilation_size = 1;

	auto const max_ed_elem = 2;
	auto const max_ed_kernel_size = 21;
	/* end */

	//cvvNamedWindow("auto threshold");

	ImageSave is("pic_x", SaveType::Image_Png, Information::Basic);

	Mat frame;
	Mat outputs[1024];
	vi pix_planarMap[1024]; // shit c++11 ->
	vi nonZero;

	// capture first frame
	cap >> frame;

	vector<Point2d> test_subPix;

	auto frameSize(frame.size());
	setImageSize(frame.size());

	// configure output stuff
	for (auto i = 0; i < frameCount_; ++i)
		pix_planarMap[i].reserve(frameSize.width);

	// start the process of gathering information for set frame count

	uint64 time_begin = getTickCount();

	while (!targetReached) {

		for (auto i = 0; i < frameCount_; ++i) {

			outputs[i] = Mat::zeros(imageSize_, CV_8UC1);
			pix_planarMap[i].clear();

			cap >> frame;

			//equalizeHist(frame, frame);

			// do basic in-place binary threshold
			threshold(frame, frame, currentThreshold, 255, CV_THRESH_BINARY);

			// blur in-place
			GaussianBlur(frame, frame, blurSize, 0, 0, BORDER_DEFAULT);

			// perform some stuff
			laplace(frame);
			//c.Sobel(frame);

			// extract information from the image, and make new output based on pixel intensity mean in Y-axis for each X point
			auto generateOk = miniCalc.generatePlanarPixels(frame, outputs[i], pix_planarMap[i], test_subPix);

			if (!generateOk) {
				cout << "autoBinaryThreshold() : Failed to map elements to 2D plane for frame #" << to_string(i + 1) << " of " << to_string(frameCount_) << endl;
				break;
			}

		}

		frame = Mat::zeros(imageSize_, CV_8UC1);
		Mat lines = Mat::zeros(imageSize_, CV_8UC1);

		// merge the images to target
		for (auto i = 0; i < frameCount_; ++i) {
			addWeighted(outputs[i], alpha, lines, beta, 0.0, lines);
			add(outputs[i], frame, frame);
			//is.SaveVideoFrame(lines);
		}

		Mat output = Mat::zeros(imageSize_, CV_8UC1);

		bilateralFilter(frame, output, 1, 80, 20);

		//auto corner_image = cornerHarris_test(lines, 200);

		auto erosion_image = this->erosion(lines, /*TESTING, def = 0 */ 1, erosion_size);
		resize(erosion_image, frame, erosion_image.size() * 2, 0, 0, INTER_LANCZOS4);

		// test for highest pixel for eroded image
		auto highestPixel = frame.rows - getHighestYpixel(frame);
		cout << "Highest Y elements in eroded line : " << highestPixel << " [mm: " << to_string(miniCalc.calculatePixelToMm(highestPixel)) << "]" << endl;

		//imshow("auto threshold", frame);

		//Mat dilation = c.dilation(lines, dilation_type, dilation_size);
		//imshow(dilationWindowName, dilation);

		vi eroded_pixels;
		findNonZero(frame, eroded_pixels);

		// generate the solar ray vectors..
		//generateVectors(eroded_pixels);

		if (pixelLimit >= eroded_pixels.size()) {
			targetReached = true;
			//cout << "Saving element collection + meta data...\n";
			//savePlanarImageData("_autogenerated", eroded_pixels, frame, frame.rows - getHighestYpixel(frame));
		}
		else {
			currentThreshold++;
		}

		//if (waitKey(1) >= 0) break;

	}

	frameTime_ = getTickCount() - time_begin;

	setBinaryThreshold(currentThreshold);
	return currentThreshold;
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
