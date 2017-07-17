#pragma once
#include <opencv2/opencv.hpp>

/**
* \brief Structure to hold a pair of opencv points, supports equalization check and a couple of useful constructors
* \tparam T The type of the point pair
*/
template <class T>
class line_pair {
public:

    cv::Point_<T> p1;

    cv::Point_<T> p2;

    line_pair(T x1, T x2, T y1, T y2) {
        p1.x = x1;
        p1.y = y1;
        p2.x = x2;
        p2.y = y2;
    }

    line_pair(cv::Point_<T> p1, cv::Point_<T> p2)
        : p1(p1)
          , p2(p2) {}

    friend bool operator==(const line_pair& lhs, const line_pair& rhs) {
        return lhs.p1 == rhs.p1 && lhs.p2 == rhs.p2;
    }

    friend bool operator!=(const line_pair& lhs, const line_pair& rhs) {
        return !(lhs == rhs);
    }

};
