#pragma once
#include "BaseR.h"
#include <vector>
#include "../Util/Vec.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <unordered_map>
#include <iostream>
#include "Util/Util.h"
#include <ostream>
#include "HoughLinesR.h"

class LaserR : public BaseR<float> {

	typedef struct xLine {

		// intensity vector
		vector<unsigned char> v;

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
			v.reserve(x);
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
			os << "\nv: (" << obj.v.size() << ") {\n";
			for (auto i = 0; i < obj.v.size(); i++) os << i << ' ' << obj.v.at(i) << '\n';
			os << '}';
			return os;
		}

	} xLine;

	vector<xLine> lines_;

	bool computeXLine();

	void configureXLine(cv::Mat& image, vector<cv::Point2i>& nonZeroes, vector<v3<float>>& output);

	static void computeCut(xLine& diagonal, HoughLinesR::LineV& horizontal);

	bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f& r) const;

	bool LaserR::intersection(cv::Vec4f& o, cv::Vec2f& p, cv::Vec4f& r);

public:

	bool computeIntensityWeigth(cv::Mat& image, vector<v3<float>>& output);

};

inline bool LaserR::computeXLine() {

	auto ok = false;

	for (auto& xl : lines_) {
		if (xl.v.empty()) continue;

		auto sum = 0.0f;
		for (auto& y : xl.v) { sum += y; }
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
	for (auto& nz : nonZeroes) lines_[nz.x].v.push_back(image.at<unsigned char>(nz));

	auto ok = computeXLine();

	if (!ok) {
		Util::loge("Error while calculating weighted intensity distribution.");
		return;
	}

	for (auto& line : lines_) output.push_back(v3<float>(static_cast<float>(line.x), line.y, static_cast<float>(line.v.size())));

}

//inline float LaserR::computeIntersect(float*__restrict y, float*__restrict a, int x) { return *y - (*a * static_cast<unsigned int>(x)); }

//inline void LaserR::computeCut(xLine& diagonal, HoughLinesR::LineV& horizontal) { diagonal.cut = computeIntersect(&diagonal.y, &horizontal.slobe, diagonal.x); }

#ifdef CV_VERSION

inline bool LaserR::intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f& r) const {
	auto d1 = p1 - o1;
	auto d2 = p2 - o2;

	auto cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8) return false;

	auto x = o2 - o1;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

#endif

inline bool LaserR::computeIntensityWeigth(cv::Mat& image, vector<v3<float>>& output) {

	// accu non-zero pixels.
	vector<cv::Point2i> nonZero;
	nonZero.reserve(image.cols * image.rows);

	cv::findNonZero(image, nonZero);

	// guard
	if (nonZero.empty()) return false;

	// clear if not empty
	if (!output.empty()) output.clear();

	// reserve enough space to avoid automatic resizing
	output.reserve(nonZero.size());

	configureXLine(image, nonZero, output);

	return !output.empty();

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
