//          Copyright Rudy Alex Kohn 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <opencv2/shape/hist_cost.hpp>
#include "calc.h"
#include "tg.h"

namespace calc {

    using namespace tg;

    bool compute_intersection_points(cv::Vec4d& horizontal_line, const cv::Vec4d& left_border, const cv::Vec4d& right_border, cv::Vec4d& output) {
        // align horizontal line
        horizontal_line[0] = left_border[0];
        horizontal_line[2] = right_border[2];

        cv::Point2d left_intersection;
        cv::Point2d right_intersection;

        // check intersection for both sides
        auto intersects_left = intersection(left_border, horizontal_line, left_intersection);
        auto intersects_right = intersection(right_border, horizontal_line, right_intersection);

        // check if there are two intersections
        if (!(intersects_left & intersects_right)) {
            log_time << "Error in intersection detection.\n";
            CV_Error(cv::Error::StsAssert, "Intersection points are invalid.");
            return false;
        }

        // assign result to output vector
        output[0] = left_intersection.x;
        output[1] = left_intersection.y;
        output[2] = right_intersection.x;
        output[3] = right_intersection.y;

        return true;
    }
}
