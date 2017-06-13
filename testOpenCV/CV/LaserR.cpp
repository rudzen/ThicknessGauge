#include <opencv2/core.hpp>
#include "LaserR.h"

bool LaserR::doLaser() {

}

bool LaserR::computeIntensityWeigth(vector<v3<float>>& output) {

    // accu non-zero pixels.
    vector<cv::Point2i> nonZero;
    nonZero.reserve(image_.cols * image_.rows);

    cv::findNonZero(image_, nonZero);

    // guard
    if (nonZero.empty())
        return false;

    // clear if not empty
    if (!output.empty())
        output.clear();

    // reserve enough space to avoid automatic resizing
    output.reserve(nonZero.size());

    configureXLine(nonZero, output);

    return !output.empty();

}
