#pragma once
#include <opencv2/core/mat.hpp>

namespace filters {

    const cv::Mat kernel_line_left_to_right = (cv::Mat_<char>(4, 4) <<
        0 , 0 , 1 , 1 ,
        0 , 1 , 1 , 1 ,
        1 , 1 , 1 , 0 ,
        1 , 1 , 0 , 0
    );

    const cv::Mat kernel_line_right_to_left = (cv::Mat_<char>(4, 4) <<
        1 , 0 , 0 , 0 ,
        1 , 1 , 1 , 0 ,
        0 , 1 , 1 , 1 ,
        0 , 0 , 0 , 1
    );

    const cv::Mat kernel_horizontal_line = (cv::Mat_<char>(4, 1) <<
        0 ,
        1 ,
        1 ,
        0
    );

}
