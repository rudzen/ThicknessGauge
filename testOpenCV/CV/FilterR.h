#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/shape/hist_cost.hpp>

/*
(      -4QQQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
(        4QQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
( )QQQm. ]QQQQQQQQQQQQQQQQQ: jQQQQQQQQQQ
( )WQQQ; ]Qf =QQQ  dQQ@^ -4: jQ(      QQ
( )WQQD  jQf =QQQ  dQW`  .   jQc___   QQ
(       jWQf =QQQ  dQf .mQc  jQQQQF  jQQ
(       ?WQf =QQQ  dQ; ]QQQ  jQQQP  jWQQ
( )WQQL  WWf =QQQ  dQ: jQQQ. jQQD  <QWQQ
( )WQQW  dQf :QQW  dQ; )QQ@  jQ@` _QQQQQ
( )WQQm  3Qk  ??'  dQL  "T'  jQ'  TTTTQQ
( )WQQQ  3QQ,   <  dQQ,   _. jQ       WW
wawWQQQwaaQQQwawWaamQQmc_wmwayQaaaaaaamQ
QWWQQQQWWQQQQQWQQQWQQQQQQQQWWQQQWQWQWQQQ
*/

/**
 * \brief Generic filter class with support for live changing of parameters
 */
class FilterR {

	cv::Mat original_;

	cv::Mat image_;

	cv::Mat result_;

	cv::Mat kernel_;

	cv::Point anchor_;

	double delta_;

	int ddepth_;

	int border_;

public:

	FilterR(): delta_(0.0)
	         , ddepth_(0)
	         , border_(cv::BORDER_DEFAULT) {
		generateKernel(3, 3, 1.0f);
		anchor_ = cv::Point(-1, -1);
	}

	FilterR(const cv::Mat& original, const cv::Mat& image, int ddepth, cv::Mat kernel, const cv::Point& anchor, double delta, int border) : original_(original)
	                                                                                                                                      , image_(image)
	                                                                                                                                      , kernel_(kernel)
	                                                                                                                                      , anchor_(anchor)
	                                                                                                                                      , delta_(delta)
	                                                                                                                                      , ddepth_(ddepth)
	                                                                                                                                      , border_(border) { }

	void generateKernel(int width, int height, float modifier);

	void doFilter();

	void doFilter(int depth);

	void doFilter(int depth, cv::Mat& kernel);

	void doFilter(int depth, cv::Mat& kernel, cv::Point& anchor);

	void doFilter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta);

	void doFilter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta, int border);

};

inline void FilterR::generateKernel(int width, int height, float modifier) { kernel_ = cv::Mat::ones(width, height, CV_32F) / (static_cast<float>(width * height) * modifier); }

inline void FilterR::doFilter() { doFilter(ddepth_, kernel_, anchor_, delta_, border_); }

inline void FilterR::doFilter(int depth) { doFilter(depth, kernel_); }

inline void FilterR::doFilter(int depth, cv::Mat& kernel) { doFilter(depth, kernel, anchor_); }

inline void FilterR::doFilter(int depth, cv::Mat& kernel, cv::Point& anchor) { doFilter(depth, kernel, anchor, delta_); }

inline void FilterR::doFilter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta) { doFilter(depth, kernel, anchor, delta, border_); }

inline void FilterR::doFilter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta, int border) { cv::filter2D(image_, result_, depth, kernel, anchor, delta, border); }
