#include <opencv2/core.hpp>
#include "HoughLinesR.h"

void HoughLinesR::computeMeta() {

    if (allLines_.empty())
        return;

    auto size = allLines_.size();

    rightLines_.clear();
    rightLines_.reserve(size);

    leftLines_.clear();
    leftLines_.reserve(size);

    auto center = image_.cols / 2;

    using namespace tg;

    for (auto& a : allLines_) {
        a.slobe = calc::slope(a.entry[0], a.entry[2], a.entry[1], a.entry[3]);
        // TODO : do something in regards to the slobe
        //log_time << cv::format("slobe calc : %f\n", a.slobe);
        if (a.points.first.x < center)
            leftLines_.emplace_back(a);
        else
            rightLines_.emplace_back(a);
    }

    auto lSize = leftLines_.size();
    auto rSize = rightLines_.size();

    // TODO : replace with throw asserts ?
    if (lSize + rSize == 0)
        throw NoLineDetectedException("No marking lines detected.");

    if (lSize == 0)
        throw NoLineDetectedException("No marking left line detected.");

    if (rSize == 0)
        throw NoLineDetectedException("No marking right line detected.");

    for (auto& left : leftLines_) {
        cv::LineIterator it(image_, left.points.first, left.points.second, 8);
        left.elements.reserve(it.count);
        for (auto i = 0; i < it.count; i++ , ++it)
            left.elements.emplace_back(it.pos());
    }

    if (lSize > 1)
        sort(leftLines_.begin(), leftLines_.end(), lineVsizeSort);

    for (auto& right : rightLines_) {
        cv::LineIterator it(image_, right.points.first, right.points.second, 8);
        right.elements.reserve(it.count);
        for (auto i = 0; i < it.count; i++ , ++it)
            right.elements.emplace_back(it.pos());
    }

    if (rSize > 1)
        sort(rightLines_.begin(), rightLines_.end(), lineVsizeSort);

}
