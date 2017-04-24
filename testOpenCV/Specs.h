#pragma once


class Specs {

public:
	
	Specs() {
	};

private:

	//const double pixMm_ = 3.45 *0.1;
	const double pixMm_ = 3.45 * 0.001;

	const int defaultHeight_ = 2448;

	const int defaultWidth_ = 2050;

	const int defaultAreaInPixels_ = defaultHeight_ * defaultWidth_;


	typedef struct {
		int offset_x : 4;
		int offset_y : 4;
		int pos_x : 4;
		int pos_y : 4;
	} offset;

	offset offset_;


public:
	double PixMm() const;
	int DefaultHeight() const;
	int DefaultWidth() const;

	double getPixelStrengths(cv::Mat& image, vi& pixels, int x);

	double getNonBaseLine(cv::Mat& image, int baseLine);

};

inline double Specs::PixMm() const {
	return pixMm_;
}

inline int Specs::DefaultHeight() const {
	return defaultHeight_;
}

inline int Specs::DefaultWidth() const {
	return defaultWidth_;
}

inline double Specs::getPixelStrengths(cv::Mat& image, vi& pixels, int x) {

	MiniCalc miniCalc;

	auto data = reinterpret_cast<float*>(image.data);

	auto sum = cv::sum(image.col(x))[0];

	auto avg = sum / image.col(x).size().height;

	cout << std::to_string(sum) << " : " << avg << endl;

	return 0.0;
}

inline double Specs::getNonBaseLine(cv::Mat& image, int baseLine) {

	MiniCalc miniCalc;
	const auto COLUMN = 0;


	Mat row_mean, col_mean;
	reduce(image, row_mean, 0, CV_REDUCE_AVG);
	reduce(image, col_mean, 1, CV_REDUCE_AVG);

	//cout << "row mean : " << row_mean.size() << endl;
	//cout << "col mean : " << col_mean.size().height << endl;

	return col_mean.size().height;

}
