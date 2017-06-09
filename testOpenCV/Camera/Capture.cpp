#include "Capture.h"

void Capture::retrieveAllInfo() {
	settings->brightness = cap.get(CV_CAP_PROP_BRIGHTNESS);
	settings->contrast = cap.get(CV_CAP_PROP_CONTRAST);
	settings->saturation = cap.get(CV_CAP_PROP_SATURATION);
	settings->hue = cap.get(CV_CAP_PROP_HUE);
	settings->gain = cap.get(CV_CAP_PROP_GAIN);
	settings->exposure = cap.get(CV_CAP_PROP_EXPOSURE);
	settings->Rgb = cap.get(CV_CAP_PROP_CONVERT_RGB);
	settings->white_balance_u = cap.get(CV_CAP_PROP_WHITE_BALANCE_BLUE_U);
	settings->white_balance_v = cap.get(CV_CAP_PROP_WHITE_BALANCE_RED_V);
	settings->rectification = cap.get(CV_CAP_PROP_RECTIFICATION);
	settings->iso_speed = cap.get(CV_CAP_PROP_ISO_SPEED);
	settings->buffersize = cap.get(CV_CAP_PROP_BUFFERSIZE);
}

double Capture::detectExposure() {
	cv::Mat t;
	cv::Scalar mean;
	cv::Scalar stddev(0.0, 0.0, 0.0);

	auto exposure = 0.0;

	cap.set(CV_CAP_PROP_EXPOSURE, exposure);

	const cv::Vec2d target(targetStdDev_ - delta_, targetStdDev_ + delta_);

	// TODO : Få lortet til at virke!

	while (stddev[0] < target[0]) {
		exposure += 100.0;
		cap.set(CV_CAP_PROP_EXPOSURE, exposure);
		cap >> t;
		cv::meanStdDev(t, mean, stddev);

		std::cout << cv::format("trying [exposure:%f] [mean:%f] [stddev:%f]\n", cap.get(CV_CAP_PROP_EXPOSURE), mean[0], stddev[0]);

		imwrite(cv::format("images/exposuretest_%f.png", exposure), t);


	}

	std::cout << cv::format("BINGO [exposure:%f] [mean:%f] [stddev:%f]\n", exposure, mean[0], stddev[0]);

	return exposure;
}
