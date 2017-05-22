#pragma once
#include "BaseR.h"
#include <vector>
#include "../Util/Vec.h"
//#include <opencv2/core/mat.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <unordered_map>
#include <iostream>
#include "Util/Util.h"
#include <ostream>
#include "HoughLinesR.h"

class LaserR : public BaseR<float> {

	typedef struct xLine {

		// intensity vector
		vector<unsigned char> i;

		// the x pos
		int x;

		// the calculated avg y pos
		float y;

		// where x is zero
		float cut;

		xLine() {
			x = 0;
			y = 0.0f;
			cut = 0.0f;
		}

		explicit xLine(int x) : x(x) {
			y = 0.0f;
			cut = 0.0f;
			i.reserve(x);
		}

		friend bool operator==(const xLine& lhs, const xLine& rhs) { return lhs.y == rhs.y; }

		friend bool operator!=(const xLine& lhs, const xLine& rhs) { return !(lhs == rhs); }

		friend bool operator<(const xLine& lhs, const xLine& rhs) { return lhs.y < rhs.y; }

		friend bool operator<=(const xLine& lhs, const xLine& rhs) { return !(rhs < lhs); }

		friend bool operator>(const xLine& lhs, const xLine& rhs) { return rhs < lhs; }

		friend bool operator>=(const xLine& lhs, const xLine& rhs) { return !(lhs < rhs); }

		friend std::ostream& operator<<(std::ostream& os, const xLine& obj) {
			os << "x: " << obj.x;
			os << " y: " << obj.y;
			os << " cut:" << obj.cut;
			os << "\nv: (" << obj.i.size() << ") {\n";
			for (auto i = 0; i < obj.i.size(); i++) os << i << ' ' << obj.i.at(i) << '\n';
			os << '}';
			return os;
		}

		const cv::Point2f& xy() const {
			return cv::Point2f(static_cast<float>(x), y);
		}

	} xLine;

	vector<xLine> lines_;

	bool computeXLine();

	void configureXLine(cv::Mat& image, vector<cv::Point2i>& nonZeroes, vector<v3<float>>& output);

	static void computeCut(xLine& diagonal, HoughLinesR::LineV& horizontal);

public:

	bool computeIntensityWeigth(cv::Mat& image, vector<v3<float>>& output);

};

inline bool LaserR::computeXLine() {

	auto ok = false;

	for (auto& xl : lines_) {
		if (xl.i.empty()) continue;

		auto sum = 0.0f;
		for (auto& intensity : xl.i) { sum += xl.i.size() * intensity; }
		xl.y = 100.0f / sum;
		ok = true;
		cout << "x / y: " << xl.x << " / " << xl.y << endl;
	}

	return ok;

}

inline void LaserR::configureXLine(cv::Mat& image, vector<cv::Point2i>& nonZeroes, vector<v3<float>>& output) {

	if (!lines_.empty()) lines_.clear();

	lines_.reserve(image.cols);

	// populate xLine vector
	for (auto i = 0; i < image.cols; i++) lines_.push_back(xLine(i));

	// copy the values from nonZero vector
	for (auto& nz : nonZeroes) lines_[nz.x].i.push_back(image.at<unsigned char>(nz));

	auto ok = computeXLine();

	if (!ok) {
		Util::loge("Error while calculating weighted intensity distribution.");
		return;
	}

	for (auto& line : lines_) output.push_back(v3<float>(static_cast<float>(line.x), line.y, static_cast<float>(line.i.size())));

}



  /*
   |  __
   | /__\
   | X~~|			"The eternal code god
   |-\|//-.		 watches over this mess."
/|`.|'.' \			- R.A.Kohn, 2017
|,|.\~~ /||
|:||   ';||
||||   | ||
\ \|     |`.
|\X|     | |
| .'     |||
| |   .  |||
|||   |  `.| JS
||||  |   ||
||||  |   ||
`+.__._._+*/
