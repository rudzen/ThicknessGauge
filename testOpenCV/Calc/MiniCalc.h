#pragma once

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "LineConfig.h"
#include "../Util/Vec.h"

#include "../namespaces/tg.h"
#include "namespaces/sort.h"
#include "namespaces/cvR.h"

using namespace std;
using namespace tg;

class MiniCalc {

public:
    MiniCalc();
    ~MiniCalc();

    /* Converts pixels from an image to a singular line vector in X, where Y is the mean of all pixels located at that given X in the image */
    bool generatePlanarPixels(cv::Mat& input, cv::Mat& output, vector<cv::Point2f>& pixels, vector<cv::Point2f>& gradientPixels) const;










};
