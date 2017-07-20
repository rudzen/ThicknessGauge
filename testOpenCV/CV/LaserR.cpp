#include <opencv2/core.hpp>
#include "LaserR.h"

bool LaserR::do_laser() {
    return false;
}

bool LaserR::compute_intensity_weigth(vector<v3<float>>& output) {

    // accu non-zero pixels.
    vector<cv::Point2i> non_zeroes;
    non_zeroes.reserve(image_.cols * image_.rows);

    cv::findNonZero(image_, non_zeroes);

    // guard
    if (non_zeroes.empty()) {
        return false;
    }

    // clear if not empty
    if (!output.empty()) {
        output.clear();
    }

    // reserve enough space to avoid automatic resizing
    output.reserve(non_zeroes.size());

    configure_x_line(non_zeroes, output);

    return !output.empty();

}
