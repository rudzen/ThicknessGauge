/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 8 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#if !defined HARRISD
#define HARRISD

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

template <class T>
class HarrisDetector {

    // 32-bit float image of corner strength
    cv::Mat corner_strength_;

    // 32-bit float image of thresholded corners
    cv::Mat corner_threshold_;

    // image of local maxima (internal)
    cv::Mat local_max_;

    // size of neighbourhood for derivatives smoothing
    int neighbourhood_;

    // aperture for gradient computation
    int aperture_;

    // Harris parameter
    T k_;

    // maximum strength for threshold computation
    T max_strength_;

    // calculated threshold (internal)
    T threshold_;

    // size of neighbourhood for non-max suppression
    int non_max_size_;

    // kernel for non-max suppression
    cv::Mat kernel_;

public:

    HarrisDetector()
        : neighbourhood_(3)
        , aperture_(3)
        , k_(0.1)
        , max_strength_(0.0)
        , threshold_(0.01)
        , non_max_size_(3) {

        local_max_window_size(non_max_size_);
    }

    // Create kernel used in non-maxima suppression
    void local_max_window_size(int size) {
        non_max_size_ = size;
        kernel_.create(non_max_size_, non_max_size_,CV_8U);
    }

    // Compute Harris corners
    void detect(const cv::Mat& image) {

        // Harris computation
        cv::cornerHarris(image, corner_strength_,
                         neighbourhood_,// neighborhood size
                         aperture_, // aperture size
                         k_); // Harris parameter

        // internal threshold computation
        double min_strength; // not used
        cv::minMaxLoc(corner_strength_, &min_strength, &max_strength_);

        // local maxima detection
        cv::Mat dilated; // temporary image
        cv::dilate(corner_strength_, dilated, cv::Mat());
        cv::compare(corner_strength_, dilated, local_max_, cv::CMP_EQ);
    }

    // Get the corner map from the computed Harris values
    cv::Mat corner_map(double quality_level) {

        cv::Mat cmap;

        // thresholding the corner strength
        threshold_ = quality_level * max_strength_;
        cv::threshold(corner_strength_, corner_threshold_, threshold_, 255, cv::THRESH_BINARY);

        // convert to 8-bit image
        corner_threshold_.convertTo(cmap,CV_8U);

        // non-maxima suppression
        cv::bitwise_and(cmap, local_max_, cmap);

        return cmap;
    }

    // Get the feature points vector from the computed Harris values
    void corners(std::vector<cv::Point_<T>>& points, double quality_level) {

        // Get the corner map
        auto cmap = corner_map(quality_level);
        // Get the corners
        corners(points, cmap);
    }

    // Get the feature points vector from the computed corner map
    static void corners(std::vector<cv::Point_<T>>& points, const cv::Mat& corner_map) {

        // TODO : replace with blazing fast position iteration instead

        // Iterate over the pixels to obtain all feature points
        for (auto y = 0; y < corner_map.rows; y++) {

            auto rowPtr = corner_map.ptr<uchar>(y);

            for (auto x = 0; x < corner_map.cols; x++) {

                // if it is a feature point
                if (rowPtr[x])
                    points.emplace_back(cv::Point(x, y));
            }
        }
    }

    // Draw circles at feature point locations on an image
    static void draw_on_image(cv::Mat& image, const std::vector<cv::Point_<T>>& points, cv::Scalar color = cv::Scalar(255, 255, 255), int radius = 3, int thickness = 2) {

        for (const auto& p : points)
            cv::circle(image, p, radius, color, thickness);

    }
};

#endif
