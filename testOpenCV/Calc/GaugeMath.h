#pragma once
#include "Util/Vec.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/videostab/fast_marching.hpp>

class GaugeMath {

public:
    GaugeMath(double angle, double height, double dist, double laserDist, double focal, double ccd)
        : angle_(angle)
        , height_(height)
        , distCamToLaser_(dist)
        , distLaserToGround_(laserDist)
        , focal_(focal)
        , ccd_(ccd) {
        distCamToCenter_ = 0.0;
    }

    double computeRange(cv::Point2d& p1, cv::Point2d& p2) {
        return (height_ * focal_) / (abs(p2.y - p1.y) * ccd_);
    }

private:

    // angle of camera
    double angle_;

    // height of camera (and laser)
    double height_;

    // distance from camera to laser
    double distCamToLaser_;

    // distance from laser to ground level
    double distLaserToGround_;

    // distance from camera to center pixel
    double distCamToCenter_;

    // focal length of camera
    double focal_;

    // ccd of camera
    double ccd_;

};
