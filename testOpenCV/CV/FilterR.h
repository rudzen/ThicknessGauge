#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/shape/hist_cost.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "BaseR.h"

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
QQQQQQQ (c) 2017 Rudy Alex Kohn QQQQQQQQ
QQQQQQQ  rudyalexkohn@rudz.dk	QQQQQQQQ
*/

/**
 * \brief Generic filter class with support for live changing of parameters
 */
class FilterR : public BaseR {

	/**
	 * \brief The result of the process
	 */
	cv::Mat result_;

	/**
	 * \brief The kernel to use for processing the image
	 */
	cv::Mat kernel_;

	/**
	 * \brief The kernel anchor point.
	 * The position of the anchor relative to its kernel. The location Point(-1, -1) indicates the center by default.
	 */
	cv::Point anchor_;

	/**
	 * \brief The base modifier value to add (or substract) to each value in the kernel.
	 */
	double delta_;

	/**
	 * \brief The depth of result_. A negative value (such as -1) indicates that the depth is the same as the source.
	 */
	int ddepth_;

	/**
	 * \brief The border to use in the process.
	 */
	int border_;

	/**
	 * \brief Show the result window
	 */
	bool showWindow_;

	// only for UI

	int deltaI_ = 0;

	void createWindow() {
		cv::namedWindow(windowName, cv::WINDOW_KEEPRATIO);
		cv::createTrackbar("delta", windowName, &deltaI_, 3, delta_cb, this);
	}

	static void delta_cb(int value, void* userData) {
		auto that = static_cast<FilterR*>(userData);
		auto oldVal = that->getDelta();
		that->setDelta(static_cast<double>(value));
		std::cout << that->windowName << " delta : " << oldVal << " -> " << value << '\n';
	}

public: // constructors

	explicit FilterR(std::string windowName): delta_(0.0)
	         , ddepth_(-1)
	         , border_(cv::BORDER_DEFAULT)
	         , showWindow_(true) {
		generateKernel(3, 3, 1.0f);
		anchor_ = cv::Point(-1, -1);
		this->windowName = windowName;
		createWindow();
	}

	FilterR(const cv::Mat& original, const cv::Mat& image, int ddepth, cv::Mat kernel, const cv::Point& anchor, double delta, int border, bool showWindows, std::string windowName) : kernel_(kernel)
	                                                                                                                                                        , anchor_(anchor)
	                                                                                                                                                        , delta_(delta)
	                                                                                                                                                        , ddepth_(ddepth)
	                                                                                                                                                        , border_(border)
	                                                                                                                                                        , showWindow_(showWindows)
	{
		this->windowName = windowName;
		if (showWindows) createWindow();
	}

public: // getters & setters

	cv::Mat& getResult() { return result_; }

	const cv::Mat& getKernel() const { return kernel_; }

	void setKernel(const cv::Mat& kernel) { kernel_ = kernel; }

	const cv::Point& getAnchor() const { return anchor_; }

	void setAnchor(const cv::Point& anchor) { anchor_ = anchor; }

	double getDelta() const { return delta_; }

	void setDelta(double delta) { delta_ = delta; }

	int getDdepth() const { return ddepth_; }

	void setDdepth(int ddepth) { ddepth_ = ddepth; }

	int getBorder() const { return border_; }

	void setBorder(int border) { border_ = border; }

	bool getShowWindow() const { return showWindow_; }

	void setShowWindow(bool showWindow) { showWindow_ = showWindow; }

	// functions

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

inline void FilterR::doFilter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta, int border) {
	cv::filter2D(image_, result_, depth, kernel, anchor, delta, border);
	if (getShowWindow())
		cv::imshow(windowName, result_);
}
