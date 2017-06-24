#include <opencv2/core.hpp>
#include "HoughLinesR.h"

bool HoughLinesR::is_lines_intersecting(Side side) {

    auto& lines = side == Side::Right ? right_lines_ : left_lines_;

    if (lines.empty())
        return false;

    auto size = lines.size();

    if (size == 1)
        return true;

    auto intersects = true;

    auto& first = lines.front();

    for (size_t i = 1; i < size; ++i) {
        auto& p1 = lines[i].points.p1;
        auto& p2 = lines[i].points.p2;
        intersects = calc::intersection(first.points.p1, p1, first.points.p2, p2);
        if (!intersects)
            return false;
    }

    return intersects;

}

void HoughLinesR::compute_meta() {

    if (all_lines_.empty())
        return;

    auto size = all_lines_.size();

    right_lines_.clear();
    right_lines_.reserve(size);

    left_lines_.clear();
    left_lines_.reserve(size);

    auto center = image_.cols / 2;

    using namespace tg;

    //log_time << __FUNCTION__ << " center: " << center << std::endl;

    for (auto& a : all_lines_) {
        // TODO : do something in regards to the slobe
        a.slobe = calc::slope(a.entry_[0], a.entry_[2], a.entry_[1], a.entry_[3]);
        if (a.points.p1.x > center) {
            //log_time << __FUNCTION__ << " right point added : " << a.points.p1 << std::endl;
            right_lines_.emplace_back(a);
        } else {
            //log_time << __FUNCTION__ << " left point added : " << a.points.p1 << std::endl;
            left_lines_.emplace_back(a);
        }
    }

    auto lSize = left_lines_.size();
    auto rSize = right_lines_.size();

    //// TODO : replace with throw asserts ?
    //if (lSize + rSize == 0)
    //    throw NoLineDetectedException("No marking lines detected.");

    //if (lSize == 0)
    //    throw NoLineDetectedException("No marking left line detected.");

    //if (rSize == 0)
    //    throw NoLineDetectedException("No marking right line detected.");

    for (auto& left : left_lines_) {
        cv::LineIterator it(image_, left.points.p1, left.points.p2, 8);
        left.elements.reserve(it.count);
        for (auto i = 0; i < it.count; i++ , ++it)
            left.elements.emplace_back(it.pos());
    }

    auto line_sort = [](const LineV& l1, const LineV& l2) { return l1.elements.size() < l2.elements.size(); };

    if (lSize > 1)
        std::sort(left_lines_.begin(), left_lines_.end(), line_sort);

    for (auto& right : right_lines_) {
        cv::LineIterator it(image_, right.points.p1, right.points.p2, 8);
        right.elements.reserve(it.count);
        for (auto i = 0; i < it.count; i++ , ++it)
            right.elements.emplace_back(it.pos());
    }

    if (rSize > 1)
        std::sort(right_lines_.begin(), right_lines_.end(), line_sort);

}
