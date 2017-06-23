#pragma once
#include <opencv2/core/mat.hpp>
#include "BaseR.h"
#include "namespaces/tg.h"

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

    // only for UI

    int deltaI_ = 0;

    void create_window();

    static void delta_cb(int value, void* user_data);

public: // constructors

    explicit FilterR(std::string window_name, bool show_windows);

    explicit FilterR(std::string window_name);

    FilterR(const cv::Mat& original, const cv::Mat& image, int ddepth, cv::Mat kernel, const cv::Point& anchor, double delta, int border, bool show_windows, std::string window_name);

    // getters & setters

    cv::Mat& result();

    const cv::Mat& kernel() const;

    void kernel(const cv::Mat& new_kernel);

    const cv::Point& anchor() const;

    void anchor(const cv::Point& new_anchor);

    double delta() const;

    void delta(double new_delta);

    int ddepth() const;

    void ddepth(int new_ddepth);

    int border() const;

    void border(int new_border);

    // functions

    void generate_kernel(int width, int height, float modifier);

    void do_filter();

    void do_filter(int depth);

    void do_filter(int depth, cv::Mat& kernel);

    void do_filter(int depth, cv::Mat& kernel, cv::Point& anchor);

    void do_filter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta);

    void do_filter(int depth, cv::Mat& kernel, cv::Point& anchor, double delta, int border);

};
